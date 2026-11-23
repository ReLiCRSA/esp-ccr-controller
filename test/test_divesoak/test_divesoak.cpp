// ============================================================================
// test_divesoak — M6 exit criterion: 50 scripted dives with seeded fault
// injection, run on the PC simulator, with the safety invariants checked on
// every 100 ms control cycle.
//
// What makes this more than 50 copies of test_sim:
//   * profile, fault choice, injection time and the simulated diver's reaction
//     are seeded per dive: a failure is reproduced by re-running that one dive
//     number, and the suite is not a different program every morning;
//   * the asserts are invariants, not expectations — no setpoint escape, no
//     unalarmed hypoxia, no unalarmed toxicity, every injected fault noticed
//     inside a deadline *and* its safety consequence actually followed, the
//     AUTO lockout never bypassed, and the dive file that comes out the other
//     end intact and complete.
//
// Sensitivity was measured, not assumed: dropping the per-cell high veto,
// moving the high alarm to the wall value, un-binding the CellProof verdict from
// the vote, or removing the vote's fallback when a ban empties it each makes
// this suite fail. What it cannot reach: the staleness gate, the depth filter
// and the stuck-open hysteresis (the gas model never makes a sample stale or
// dither) — test_loop, test_depth and test_injector own those.
//
// The diver model is deliberately simple and fallible: it takes a human moment
// to notice an alarm, it ACKs, and it closes the manual inhibit when the
// machine says the valve is open. Faults are injected only through the world.
// ============================================================================
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "log/divelog.h"
#include "sim/rig.h"

using namespace sim;

void setUp() {}
void tearDown() {}

namespace {

// ---------------------------------------------------------------------------
// Deterministic per-dive randomness
// ---------------------------------------------------------------------------
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed * 2654435761u + 1013904223u) {}
    uint32_t next() { s = s * 1664525u + 1013904223u; return s >> 8; }
    int inRange(int lo, int hi) { return lo + (int)(next() % (uint32_t)(hi - lo + 1)); }
    float fl(float lo, float hi) { return lo + (hi - lo) * ((float)next() / 16777216.0f); }
    bool chance(int pct) { return (int)(next() % 100u) < pct; }
};

char g_msg[300];  // why a dive failed, including its number
#define FAIL(...)                                         \
    do {                                                  \
        std::snprintf(g_msg, sizeof(g_msg), __VA_ARGS__);  \
        TEST_ASSERT_TRUE_MESSAGE(false, g_msg);            \
    } while (0)

// ---------------------------------------------------------------------------
// Fault plan: a fault must be NOTICED in time and its consequence must follow
// ---------------------------------------------------------------------------
enum FaultKind : uint8_t { F_NONE, F_DEAD, F_SATURATE, F_STUCK_LOW, F_STUCK_HIGH,
                           F_SLOW, F_VALVE, F_FLUSH, F_TWIN_LIE, F_TWIN_DRIFT };

const char* faultName(FaultKind k) {
    switch (k) {
        case F_DEAD: return "cell died";
        case F_SATURATE: return "cell saturated";
        case F_STUCK_LOW: return "cell stuck low";
        case F_STUCK_HIGH: return "cell stuck high";
        case F_SLOW: return "cell sluggish";
        case F_VALVE: return "solenoid stuck open";
        case F_FLUSH: return "diluent flush";
        case F_TWIN_LIE: return "two cells agreeing low";
        case F_TWIN_DRIFT: return "two cells drifting low together";
        default: return "none";
    }
}

struct Fault {
    FaultKind kind = F_NONE;
    float atS = 0;                 // seconds after dive start
    uint8_t ch = 0;
    uint16_t detectMask = 0;       // any of these = the machine noticed
    float detectDeadlineS = 20.0f;
    // What must follow detection, and by when. FOLLOW_HIGH_ALARM is a mask test,
    // FOLLOW_RECOVERED means "the loop came back up to this ppO2".
    enum Follow : uint8_t { FOLLOW_NONE, FOLLOW_HIGH_ALARM, FOLLOW_RECOVERED };
    Follow follow = FOLLOW_NONE;
    float followDeadlineS = 0.0f;
    float followThreshold = 0.0f;
    bool injected = false;
    bool progressive = false;   // the lie grows over minutes, not in one sample
    float driftLeftS = 0.0f;    // ...for a bounded span, then it holds
    float detectSeenS = -1.0f, followSeenS = -1.0f;
};

RigOpts soakOpts() {
    RigOpts o;
    o.presentMask = 0b111;
    o.predive.warmupMs = 2000;   // real cells polarise for minutes; the sim need not
    o.inj.maxDutyPct = 40;
    o.inj.minGapMs = 400;
    o.world.loopVolumeL = 8.0f;
    o.world.sacLpm = 1.2f;
    o.world.injLpm = 60.0f;
    return o;
}

// An ACK silences a continuous alarm, but the mask keeps reporting it as
// suppressed-and-active. "Visible" means either: the diver was told.
uint16_t visibleAlarms(const Rig& rig) {
    uint16_t v = rig.alarmBits();
    const alarms::Result& r = rig.alarmRes();
    for (int b = 0; b < alarms::BIT_COUNT; b++)
        if (r.ackSuppressed[b]) v |= (uint16_t)(1u << b);
    return v;
}

// ---------------------------------------------------------------------------
// Dive profile: waypoints, then home under the machine's own ceiling. The
// simulated diver never asks to be shallower than the ceiling it is being told
// about, so deep dives spend real time on stops.
// ---------------------------------------------------------------------------
struct Diver {
    float depth = 0.0f;
    float depthM[4] = {};
    float holdS[4] = {};
    int nWp = 0;
    int wp = 0;
    float holdLeft = 0.0f;
    bool homing = false;
    float rateDown = 0.25f;  // m/s = 15 m/min
    float rateUp = 0.15f;    //       9 m/min

    bool step(float dtS, const deco::Zhl16Ccr& d, float tS) {
        float target = homing ? 0.0f : depthM[wp];
        const float ceil = (float)d.ceilingM();
        // The ceiling is a floor for ASCENT, never a reason to go deeper. A
        // diver who descends to chase a rising ceiling loads more tissue and
        // deepens it further — the soak's first version of this model did
        // exactly that and ran itself to 200 m.
        if (ceil > 0.3f && target < ceil + 0.5f && ceil + 0.5f < depth)
            target = ceil + 0.5f;

        // Rate-limited, and the target is only reached when the move actually
        // arrives: an unconditional "if (depth < target) depth = target" clamp
        // after the step teleports the diver to its waypoint, which makes the
        // whole dive instantaneous and the deco picture meaningless.
        if (depth < target) {
            depth += rateDown * dtS;
            if (depth > target) depth = target;
        } else if (depth > target) {
            depth -= rateUp * dtS;
            if (depth < target) depth = target;
        }
        if (depth < 0.0f) depth = 0.0f;

        if (!homing) {
            if (std::fabs(depth - depthM[wp]) < 0.2f) {
                holdLeft -= dtS;
                if (holdLeft <= 0.0f) {
                    wp++;
                    if (wp >= nWp) homing = true;
                }
            }
        } else if (depth <= 0.05f && ceil <= 0.3f) {
            return true;  // home, and the machine agrees
        }
        return false;
    }
};

void buildProfile(Rng& r, Diver& dv, float& injFrom, float& injTo, float minDepth) {
    const float floorM = minDepth;
    float maxDepth = r.fl(12.0f, 45.0f);
    if (maxDepth < floorM) maxDepth = floorM;
    const float bottom = r.fl(420.0f, 1500.0f);
    dv.depthM[0] = maxDepth;
    dv.holdS[0] = bottom;
    dv.nWp = 1;
    if (r.chance(55)) {  // the classic "let's go a bit deeper for a while"
        dv.depthM[1] = maxDepth + r.fl(-12.0f, 9.0f);
        if (dv.depthM[1] < 9.0f) dv.depthM[1] = 9.0f;
        if (dv.depthM[1] > 50.0f) dv.depthM[1] = 50.0f;
        if (dv.depthM[1] < floorM) dv.depthM[1] = floorM;
        dv.holdS[1] = r.fl(240.0f, 900.0f);
        dv.nWp = 2;
    }
    dv.holdLeft = dv.holdS[0];
    // Faults land while the diver is steady on a waypoint: that is when a
    // detection deadline is meaningful, and when a stuck valve is detectable.
    injFrom = maxDepth / dv.rateDown + 90.0f;
    injTo = injFrom + bottom * 0.55f;
}

void prediveHealthy(Rig& rig) {
    rig.prediveStart();
    for (int i = 0; i < 15 && !rig.predivePassed() && !rig.prediveFailed(); i++) {
        rig.run(4);
        if (rig.prediveStatus(predive::S_AIR) != predive::PASS) rig.world().changeO2Bar(0.02f);
    }
}

void injectFault(Rig& rig, Fault& f) {
    World& w = rig.world();
    const float voted = rig.vote().votedPpo2;
    switch (f.kind) {
        case F_DEAD: w.cells[f.ch].fault = CellFault::DEAD; break;
        case F_SATURATE:
            w.cells[f.ch].fault = CellFault::SATURATE;
            w.cells[f.ch].satCapBar = voted - 0.25f;  // the clip has to bite
            break;
        case F_STUCK_LOW:
            w.cells[f.ch].fault = CellFault::STUCK_AT;
            w.cells[f.ch].stuckValue = voted - 0.30f;
            break;
        case F_STUCK_HIGH:
            w.cells[f.ch].fault = CellFault::STUCK_AT;
            w.cells[f.ch].stuckValue = voted + 0.30f;
            break;
        case F_TWIN_LIE: {
            // Two cells agreeing on the SAME low value. The median becomes that
            // value, so the vote lies, demand never stops, and the odd cell out —
            // the one telling the truth — is the one the drift rule drops. The
            // vote then sits 0.45 bar low forever and control enriches the loop
            // that much above the band; only the per-cell high veto on the
            // excluded honest cell stops the valve there. 0.45, not more: the
            // plausibility gate rejects a 0.5 bar teleport, and it should.
            // This is the 2-vs-1 case a CCR cannot argue its way out of; what it
            // CAN do is shout about it and refuse to chase the lie into toxicity.
            const uint8_t other = (uint8_t)((f.ch + 1) % 3);
            w.cells[f.ch].fault = CellFault::STUCK_AT;
            w.cells[f.ch].stuckValue = voted - 0.45f;
            w.cells[other].fault = CellFault::STUCK_AT;
            w.cells[other].stuckValue = voted - 0.45f;
            break;
        }
        case F_TWIN_DRIFT: {
            // The dangerous shape: two cells whose output creeps down TOGETHER.
            // It is under the glitch gate and the dropout slope, so it is not a
            // "sensor failure" the voting layer can dismiss — the median slides
            // down with the lie, control chases it, and the one honest cell ends
            // up the odd one out. The per-cell high veto is what is left.
            const uint8_t other = (uint8_t)((f.ch + 1) % 3);
            const float now = rig.vote().votedPpo2;
            w.cells[f.ch].fault = CellFault::STUCK_AT;
            w.cells[f.ch].stuckValue = now;
            w.cells[other].fault = CellFault::STUCK_AT;
            w.cells[other].stuckValue = now;
            f.progressive = true;
            f.driftLeftS = 240.0f;
            break;
        }
        case F_SLOW: w.cells[f.ch].tauS = 9.0f; break;
        case F_VALVE: w.solenoidStuckOpen = true; break;
        case F_FLUSH:
            // A wrong-mix / diluent flush: a big drop, but bounded by what the
            // loop actually holds (a world that could reach 0.00 bar would be
            // testing the model, not the firmware).
            w.changeO2Bar(-std::min(0.70f, rig.world().ppo2True() - 0.15f));
            break;
        default: break;
    }
    f.injected = true;
}

// Did the machine see this fault? Detection has to come from the chain itself;
// a cell being dropped from the vote counts as the same news as the drift
// alarm, because that is what the diver is told about.
bool faultSignalSeen(const Rig& rig, const Fault& f, uint16_t mask) {
    if (mask == 0) return true;
    if (visibleAlarms(rig) & mask) return true;
    if ((mask & (1u << alarms::BIT_CELL_DRIFT)) &&
        !(rig.vote().activeMask & (1u << f.ch))) return true;
    return false;
}

// ---------------------------------------------------------------------------
// One dive
// ---------------------------------------------------------------------------
constexpr float kHighWall = 1.45f;  // inj::Config::highWallBar as the soak configures it
constexpr int kMaxSnaps = 2400;
constexpr int kMaxEvents = 400;

void runDive(int id) {
    Rng r((uint32_t)id * 7919u + 31u);
    RigOpts o = soakOpts();
    Rig rig(o);

    // ---- pre-dive lockout category: a poisoned cell fitted before the checks
    // must stop the run, and AUTO must then be unreachable ------------------
    if (id % 7 == 3) {
        const uint8_t ch = (uint8_t)r.inRange(0, 2);
        rig.world().cells[ch].fault = CellFault::STUCK_AT;
        rig.world().cells[ch].stuckValue = 0.21f;  // reads perfect air, hears nothing
        prediveHealthy(rig);
        if (!rig.prediveFailed())
            FAIL("dive %d: poisoned cell (air-reading, deaf) passed the pre-dive", id);
        if (rig.prediveStatus(predive::S_SOLENOID) != predive::FAIL)
            FAIL("dive %d: the pulse test did not fail on a deaf cell", id);
        if (rig.requestAuto(true))
            FAIL("dive %d: AUTO granted after a FAILED pre-dive", id);
        std::printf("dive %2d  pre-dive lockout: poisoned cell on ch%u denied AUTO\n", id, ch);
        return;
    }

    const int roll = r.inRange(0, 99);
    const int nFaults = roll < 20 ? 0 : (roll < 78 ? 1 : 2);
    Fault faults[2];
    static const FaultKind pool[] = {F_DEAD, F_SATURATE, F_STUCK_LOW, F_STUCK_HIGH,
                                     F_SLOW, F_VALVE, F_FLUSH, F_DEAD, F_VALVE,
                                     F_TWIN_LIE, F_TWIN_DRIFT, F_TWIN_DRIFT, F_FLUSH};
    // Fault planning, in two passes. Pass 1 picks the raw draws; pass 2 makes
    // sure the rig still has a channel that tells the truth. A lying PAIR moves
    // the median itself, so a second cell lie on top of it leaves all three
    // channels wrong at once — a state no 3-cell system can detect, and
    // asserting a detection there would be asserting the impossible. The
    // scenario where two cells lie and one is honest is exercised deliberately,
    // with the third cell left alone, in test_sim's two-lying-cells test.
    bool cellLie[3] = {};
    for (int i = 0; i < nFaults; i++) {
        Fault& f = faults[i];
        f.kind = pool[r.inRange(0, (int)(sizeof(pool) / sizeof(pool[0])) - 1)];
        f.ch = (uint8_t)r.inRange(0, 2);

        // Which channels would this fault make lie? (F_DEAD lies by reading 0.)
        uint8_t lieMask = 0;
        if (f.kind == F_TWIN_LIE || f.kind == F_TWIN_DRIFT)
            lieMask = (uint8_t)((1u << f.ch) | (1u << ((f.ch + 1) % 3)));
        else if (f.kind == F_STUCK_LOW || f.kind == F_STUCK_HIGH || f.kind == F_SATURATE ||
                 f.kind == F_DEAD)
            lieMask = (uint8_t)(1u << f.ch);

        uint8_t combined = 0;
        for (int c = 0; c < 3; c++)
            if (cellLie[c] || (lieMask & (1u << c))) combined |= (uint8_t)(1u << c);
        const bool keepsAnHonestCell = ((combined & 1) + ((combined >> 1) & 1) +
                                        ((combined >> 2) & 1)) < 3;
        if (lieMask && !keepsAnHonestCell) {
            // Still a hard fault, just one the machine can actually solve.
            f.kind = (f.ch & 1) ? F_VALVE : F_FLUSH;
            lieMask = 0;
        }
        if (lieMask & 1) cellLie[0] = true;
        if (lieMask & 2) cellLie[1] = true;
        if (lieMask & 4) cellLie[2] = true;
    }

    for (int i = 0; i < nFaults; i++) {
        Fault& f = faults[i];
        switch (f.kind) {
            case F_VALVE:
                f.detectMask = (uint16_t)(1u << alarms::BIT_STUCK_OPEN);
                f.detectDeadlineS = 20.0f;
                // Whether the loop goes toxic depends on how fast the diver gets
                // to the manual inhibit, so the guaranteed consequence here is
                // the machine's own: it must stop commanding the valve (checked
                // as an unconditional invariant below).
                break;
            case F_FLUSH:
                f.detectMask = (uint16_t)((1u << alarms::BIT_LOW_PPO2) | (1u << alarms::BIT_HYPOXIA));
                f.detectDeadlineS = 5.0f;
                f.follow = Fault::FOLLOW_RECOVERED;  // the injector must win
                f.followDeadlineS = 150.0f;
                f.followThreshold = 0.45f;
                break;
            case F_TWIN_LIE:
                // Two cells jumping low in one sample: that looks like what it
                // is, and the voting layer must report it as sensor trouble.
                f.detectMask = (uint16_t)(1u << alarms::BIT_CELL_DRIFT);
                f.detectDeadlineS = 20.0f;
                break;
            case F_TWIN_DRIFT:
                // Creeping together: no single sample is implausible, so the
                // two answers that save the dive are CellProof refusing to
                // believe cells that will not prove at the high setpoint
                // (SP_CLAMP), and the honest cell finally crossing the high
                // alarm. Either counts as noticing; the loop must not be toxic
                // before one of them arrives.
                f.detectMask = (uint16_t)((1u << alarms::BIT_HIGH_PPO2) |
                                         (1u << alarms::BIT_SP_CLAMP));
                // Measured: ~480 s at 52 m for a 0.45 bar / 4 min drift. CellProof
                // proves each cell once per dive, so a cell that STARTS lying
                // after that proof is not re-tested (recorded as an M6 hardening
                // item). What protects the dive in the meantime is the
                // unconditional rule above: no minute above 1.50 bar unalarmed.
                f.detectDeadlineS = 600.0f;
                break;
            case F_SLOW:
                f.detectMask = 0;  // nothing to detect: control just has to cope
                break;
            default:
                f.detectMask = (uint16_t)(1u << alarms::BIT_CELL_DRIFT);
                f.detectDeadlineS = 20.0f;
                break;
        }
    }

    prediveHealthy(rig);
    if (!rig.predivePassed()) FAIL("dive %d: pre-dive failed on a rig with no fitted faults", id);
    if (!rig.requestAuto(true)) FAIL("dive %d: AUTO refused after a passed pre-dive", id);

    rig.startDive();
    rig.markEnvelope();

    Diver dv;
    float injFrom = 0, injTo = 0;
    // A lying pair only turns into a toxicity problem where the setpoint band is
    // high enough that band + lie reaches the wall, so those dives get the
    // depth: 52 m puts the band at SP4 1.30, and a 0.45 bar lie at 1.75.
    float minDepth = 0.0f;
    for (int i = 0; i < nFaults; i++)
        if (faults[i].kind == F_TWIN_LIE || faults[i].kind == F_TWIN_DRIFT) minDepth = 52.0f;
    buildProfile(r, dv, injFrom, injTo, minDepth);
    for (int i = 0; i < nFaults; i++)
        faults[i].atS = r.fl(injFrom, injTo) + (float)i * 240.0f;

    // ---- invariants under watch -------------------------------------------
    const float ackDelayS = r.fl(4.0f, 15.0f);   // human reaction to an alarm
    const float valveDelayS = r.fl(1.5f, 4.0f);  // hand on the manual inhibit
    float lowUnalarmedS = 0.0f, highUnalarmedS = 0.0f, highVisibleS = 0.0f;
    float alarmSinceS = -1.0f, valveSeenS = -1.0f;
    uint16_t alarmUnion = 0;
    float maxTrue = 0.0f, minTrue = 99.0f;

    static logfmt::Snapshot snaps[kMaxSnaps];
    static logfmt::Event evs[kMaxEvents];
    int nSnap = 0, nEv = 0;
    float snapClock = 0.0f;
    uint32_t pulsesLastSnap = 0;
    uint16_t snapAlarm = 0;

    const float dtS = Rig::kTickMs / 1000.0f;
    bool finished = false;
    float tS = 0.0f;
    for (; tS < 12600.0f && !finished; tS += dtS) {
        for (int i = 0; i < nFaults; i++)
            if (!faults[i].injected && tS >= faults[i].atS) injectFault(rig, faults[i]);

        for (int i = 0; i < nFaults; i++) {
            Fault& f = faults[i];
            if (!f.progressive || !f.injected || f.driftLeftS <= 0.0f) continue;
            // 0.45 bar over 4 minutes: 0.003 bar/s, well under the dropout
            // slope — which is exactly why this fault is not a sensor story.
            // Then it holds: a cell drifting monotonically to negative for the
            // rest of the dive would be testing the scenario, not the firmware.
            f.driftLeftS -= dtS;
            const float step = 0.45f * std::min(dtS, f.driftLeftS + dtS) / 240.0f;
            rig.world().cells[f.ch].stuckValue -= step;
            rig.world().cells[(uint8_t)((f.ch + 1) % 3)].stuckValue -= step;
        }

        finished = dv.step(dtS, rig.deco(), tS);
        rig.world().setDepthM(dv.depth);
        rig.tick();

        const uint16_t bits = rig.alarmBits();
        const uint16_t seen = visibleAlarms(rig);
        const alarms::Result& ar = rig.alarmRes();
        alarmUnion |= bits;
        const float pTrue = rig.world().ppo2True();
        if (pTrue > maxTrue) maxTrue = pTrue;
        if (pTrue < minTrue) minTrue = pTrue;

        for (int i = 0; i < nFaults; i++) {
            Fault& f = faults[i];
            if (!f.injected) continue;
            if (f.detectSeenS < 0.0f && faultSignalSeen(rig, f, f.detectMask)) f.detectSeenS = tS;
            if (f.follow == Fault::FOLLOW_HIGH_ALARM && f.followSeenS < 0.0f &&
                (seen & (1u << alarms::BIT_HIGH_PPO2))) f.followSeenS = tS;
            if (f.follow == Fault::FOLLOW_RECOVERED && f.followSeenS < 0.0f &&
                rig.vote().votedPpo2 >= f.followThreshold) f.followSeenS = tS;
        }

        // ---- nothing dangerous may go unsaid ------------------------------
        // The vote's own contract, exactly: if what the machine measures is
        // below the low threshold, LOW_PPO2 is on the display. No slack.
        if (rig.vote().votedPpo2 < 0.18f && rig.loop().haveVote() &&
            !(seen & ((1u << alarms::BIT_LOW_PPO2) | (1u << alarms::BIT_HYPOXIA))))
            FAIL("dive %d: vote at %.2f bar with no low-ppO2 alarm (t=%.0fs alarms=0x%03x)",
                 id, rig.vote().votedPpo2, tS, alarmUnion);

        // The true loop ppO2 is not something the machine can measure directly,
        // so unalarmed hypoxia is a failure UNLESS the machine is simultaneously
        // shouting that its sensing cannot be trusted (CELL_DRIFT / SENSOR_FAIL /
        // SENSOR_STALE). That is the alarm which exists for exactly this case,
        // and in the water it means the same thing: stop trusting the readout.
        if (pTrue < 0.18f) {
            lowUnalarmedS += dtS;
            const bool sensorHonest = seen & ((1u << alarms::BIT_CELL_DRIFT) |
                                              (1u << alarms::BIT_SENSOR_FAIL) |
                                              (1u << alarms::BIT_SENSOR_STALE));
            if (lowUnalarmedS > 3.0f &&
                !(seen & ((1u << alarms::BIT_LOW_PPO2) | (1u << alarms::BIT_HYPOXIA))) &&
                !sensorHonest)
                FAIL("dive %d: loop at %.2f bar for %.1f s with no low/hypoxia alarm "
                     "and no sensor warning (t=%.0fs depth=%.1fm alarms=0x%03x)",
                     id, pTrue, lowUnalarmedS, tS, dv.depth, alarmUnion);
        } else lowUnalarmedS = 0.0f;

        // Absolute floor: however confused the sensing gets, the simulated diver
        // must never be left breathing a genuinely hypoxic loop at depth.
        if (pTrue < 0.12f && dv.depth > 3.0f)
            FAIL("dive %d: loop fell to %.2f bar at %.1f m (t=%.0fs alarms=0x%03x)",
                 id, pTrue, dv.depth, tS, alarmUnion);

        // 1.50, not the 1.45 alarm threshold: a cycle of slack for the vote to
        // catch up. Anything above this unalarmed is the machine failing to see
        // a cell that is plainly in the toxicity band.
        // Same rule as the low side: the machine is answerable for what it
        // measures. If a cell can see the toxicity it must say so; if no fitted
        // cell can see it, the machine's obligation is to be shouting that its
        // sensing cannot be trusted — and both are checked below, separately.
        float maxCell = 0.0f, minCell = 99.0f;
        for (int c = 0; c < 3; c++) {
            const float v = rig.cellStatus((uint8_t)c).last;
            if (v > maxCell) maxCell = v;
            if (v < minCell) minCell = v;
        }
        if (pTrue > 1.50f) {
            highUnalarmedS += dtS;
            const bool aCellCouldSeeIt = maxCell > 1.40f;
            const bool sensorHonest = seen & ((1u << alarms::BIT_CELL_DRIFT) |
                                              (1u << alarms::BIT_SENSOR_FAIL) |
                                              (1u << alarms::BIT_SENSOR_STALE));
            if (highUnalarmedS > 3.0f && !(seen & (1u << alarms::BIT_HIGH_PPO2))) {
                if (aCellCouldSeeIt)
                    FAIL("dive %d: loop at %.2f bar for %.1f s, a cell read %.2f, and the "
                         "high alarm stayed off (t=%.0fs depth=%.1fm alarms=0x%03x)",
                         id, pTrue, highUnalarmedS, maxCell, tS, dv.depth, alarmUnion);
                if (!sensorHonest && pTrue > 1.80f)
                    FAIL("dive %d: every cell agreed below the loop's %.2f bar and nothing "
                         "was said (t=%.0fs depth=%.1fm topCell=%.2f alarms=0x%03x)",
                         id, pTrue, tS, dv.depth, maxCell, alarmUnion);
            }
        } else highUnalarmedS = 0.0f;

        // The per-cell veto, asserted where it can be observed: if ANY fitted
        // cell — even one the voting layer has dropped — is at or above the
        // wall, no new pulse may leave the valve. This is the assertion that
        // fails when the veto is removed from the chain.
        if (maxCell >= kHighWall && rig.injOut().pulseStarted)
            FAIL("dive %d: pulse started while a cell read %.2f bar (wall %.2f) "
                 "at t=%.0fs vote=%.2f alarms=0x%03x",
                 id, maxCell, kHighWall, tS, rig.vote().votedPpo2, alarmUnion);

        // Hard envelope. A stuck valve can physically outrun the machine, which
        // is why the bench needs a mechanical inhibit; 2.2 bar is as far as the
        // simulated diver lets it go before closing it.
        if (pTrue > 2.20f)
            FAIL("dive %d: loop reached %.2f bar (toxic) at t=%.0fs depth=%.1fm alarms=0x%03x",
                 id, pTrue, tS, dv.depth, alarmUnion);

        // A valve already suspected open must never also be commanded open:
        // latching has to mean something at the pin, not just on the display.
        if (rig.injOut().stuckOpenLatched && rig.loop().solenoid())
            FAIL("dive %d: valve still commanded while latched stuck-open (t=%.0fs)",
                 id, tS);

        // ---- setpoint escape ----------------------------------------------
        const float sp = rig.spTarget();
        if (sp < 0.39f || sp > 1.31f)
            FAIL("dive %d: setpoint escaped to %.2f bar at t=%.0fs depth=%.1fm",
                 id, sp, tS, dv.depth);

        // ---- the simulated diver -----------------------------------------
        if (bits && alarmSinceS < 0.0f) alarmSinceS = tS;
        if (!bits) alarmSinceS = -1.0f;
        if (alarmSinceS >= 0.0f && tS - alarmSinceS >= ackDelayS) {
            rig.ackAlarms();
            alarmSinceS = -1.0f;
        }
        // A diver who is told the loop is toxic and cannot fix it terminates the
        // dive: ascent from here on, stops taken as the machine demands them.
        highVisibleS = (seen & (1u << alarms::BIT_HIGH_PPO2)) ? highVisibleS + dtS : 0.0f;
        if (highVisibleS > 120.0f && !dv.homing) {
            dv.wp = dv.nWp;
            dv.homing = true;
        }

        const bool valveAlarmed =
            (bits & ((1u << alarms::BIT_STUCK_OPEN) | (1u << alarms::BIT_HIGH_PPO2))) != 0;
        if (valveAlarmed && valveSeenS < 0.0f) valveSeenS = tS;
        if (valveSeenS >= 0.0f && tS - valveSeenS >= valveDelayS) {
            rig.world().solenoidStuckOpen = false;  // manual inhibit closed
            if (rig.loop().outputFuseTripped() || rig.injOut().stuckOpenLatched)
                rig.clearStuckOpen();
            valveSeenS = -1.0f;
        }

        // ---- dive file ----------------------------------------------------
        snapAlarm |= bits;
        if (tS >= snapClock && nSnap < kMaxSnaps) {
            snapClock = tS + 10.0f;
            logfmt::Snapshot& s = snaps[nSnap++];
            s.tSec = (uint32_t)tS;
            s.depthCm = (int32_t)(dv.depth * 100.0f);
            s.votedX1000 = (int16_t)(rig.vote().votedPpo2 * 1000.0f);
            for (int c = 0; c < 3; c++)
                s.cellX1000[c] = (int16_t)(rig.cellStatus((uint8_t)c).last * 1000.0f);
            s.setpointX1000 = (int16_t)(sp * 1000.0f);
            s.mode = rig.autoMode() ? 1 : 0;
            s.alarmBits = snapAlarm;  // anything active since the previous row
            s.cnsPct = (uint8_t)(rig.cnsPct() > 255.0 ? 255.0 : rig.cnsPct());
            s.ceilingCm = (int16_t)(rig.deco().ceilingM() * 100.0);
            const int ndl = (int)rig.deco().ndlMinutes(rig.world().pAmbBar(),
                                                       rig.vote().votedPpo2);
            s.ndlMin = (int16_t)(ndl > 480 ? 480 : (ndl < 0 ? 0 : ndl));
            s.decoTotalMin = (int16_t)rig.deco().schedule(rig.world().pAmbBar()).totalTimeMin;
            s.pulses = (uint16_t)(rig.pulseCount() - pulsesLastSnap);
            pulsesLastSnap = rig.pulseCount();
            s.battX1000 = 12400;
            s.tempC = 24;
            s.flags = (uint8_t)((rig.cpClampActive() ? 1 : 0) | ((uint8_t)rig.vote().mode << 1));
            snapAlarm = 0;
        }
        for (uint8_t e = 0; e < ar.edgeCount && nEv < kMaxEvents; e++)
            evs[nEv++] = logfmt::Event{
                (uint32_t)tS,
                ar.edges[e].raised ? logfmt::EVT_ALARM_RAISED : logfmt::EVT_ALARM_CLEARED,
                (uint16_t)ar.edges[e].id};
    }
    rig.endDive();

    std::printf("dive %2d  %5.0f m  %5.0f min  snaps %4d  ev %3d  pulses %4u  CNS %3.0f%%  "
                "ppO2 %.2f-%.2f  alarms 0x%03x  faults: %s%s\n",
                id, dv.depthM[0], tS / 60.0f, nSnap, nEv, (unsigned)rig.pulseCount(),
                rig.cnsPct(), minTrue, maxTrue, alarmUnion,
                nFaults > 0 ? faultName(faults[0].kind) : "-",
                nFaults > 1 ? faultName(faults[1].kind) : "");

    // ---- every injected fault: noticed in time, and acted on --------------
    for (int i = 0; i < nFaults; i++) {
        const Fault& f = faults[i];
        if (f.detectMask && f.detectSeenS < 0.0f)
            FAIL("dive %d: %s injected at t=%.0fs was never detected (alarms=0x%03x)",
                 id, faultName(f.kind), f.atS, alarmUnion);
        if (f.detectMask && f.detectSeenS - f.atS > f.detectDeadlineS)
            FAIL("dive %d: %s took %.0fs to detect (deadline %.0fs)",
                 id, faultName(f.kind), f.detectSeenS - f.atS, f.detectDeadlineS);
        if (f.follow != Fault::FOLLOW_NONE &&
            (f.followSeenS < 0.0f || f.followSeenS - f.atS > f.followDeadlineS))
            FAIL("dive %d: %s never produced its expected consequence "
                 "(seen=%.0fs, expected within %.0fs)",
                 id, faultName(f.kind), f.followSeenS, f.followDeadlineS);
    }

    // ---- a fault-free dive must stay quiet --------------------------------
    if (nFaults == 0 && (alarmUnion & (uint16_t)~(1u << alarms::BIT_CNS_HIGH)) != 0)
        FAIL("dive %d: clean dive raised alarms 0x%03x (max %.0f m, CNS %.0f%%)",
             id, alarmUnion, dv.depthM[0], rig.cnsPct());

    // ---- CNS stays inside the plan ---------------------------------------
    if (rig.cnsPct() >= 100.0)
        FAIL("dive %d: CNS reached %.0f%% on a %.0f m plan", id, rig.cnsPct(), dv.depthM[0]);

    // ---- dive file integrity ---------------------------------------------
    if (nSnap < 10) FAIL("dive %d: only %d snapshots logged", id, nSnap);
    if (nSnap >= kMaxSnaps) FAIL("dive %d: snapshot buffer saturated at %d rows", id, nSnap);
    for (int i = 0; i < nSnap; i++) {
        uint8_t bytes[logfmt::SNAPSHOT_SIZE];
        logfmt::packSnapshot(snaps[i], bytes);
        logfmt::Snapshot back{};
        if (!logfmt::unpackSnapshot(bytes, back))
            FAIL("dive %d: snapshot row %d failed to decode", id, i);
        if (back.alarmBits != snaps[i].alarmBits || back.tSec != snaps[i].tSec ||
            back.depthCm != snaps[i].depthCm || back.setpointX1000 != snaps[i].setpointX1000)
            FAIL("dive %d: snapshot row %d did not survive the wire (0x%03x -> 0x%03x)",
                 id, i, snaps[i].alarmBits, back.alarmBits);
    }
    for (int i = 0; i < nEv; i++) {
        uint8_t bytes[logfmt::EVENT_SIZE];
        logfmt::packEvent(evs[i], bytes);
        logfmt::Event back{};
        if (!logfmt::unpackEvent(bytes, back) || back.id != evs[i].id ||
            back.value != evs[i].value)
            FAIL("dive %d: event %d did not survive the wire", id, i);
    }

    logfmt::DiveMeta meta{};
    meta.durationSec = (uint32_t)tS;
    meta.maxDepthCm = (int32_t)(dv.depthM[0] * 100.0f);
    meta.minVotedX1000 = (int16_t)(rig.minVotedSeen * 1000.0f);
    meta.maxVotedX1000 = (int16_t)(rig.maxVotedSeen * 1000.0f);
    meta.totalPulses = (uint16_t)rig.pulseCount();
    meta.cnsPeakPct = (uint8_t)(rig.cnsPct() > 255.0 ? 255.0 : rig.cnsPct());
    meta.gfLow = 30; meta.gfHigh = 85;
    meta.sp1X1000 = 400; meta.sp2X1000 = 700; meta.sp3X1000 = 1000; meta.sp4X1000 = 1300;
    meta.eventCount = (uint16_t)nEv;
    meta.snapshotCount = (uint16_t)nSnap;
    uint8_t hdr[64];
    logfmt::packMeta(meta, hdr);
    logfmt::DiveMeta back{};
    if (!logfmt::unpackMeta(hdr, back) || back.durationSec != meta.durationSec ||
        back.snapshotCount != meta.snapshotCount)
        FAIL("dive %d: dive header did not survive the wire", id);
    hdr[20] ^= 0x40;  // one bit flipped on flash
    if (logfmt::unpackMeta(hdr, back)) FAIL("dive %d: header CRC did not catch corruption", id);

    // The export a diver actually reads: rebuild the tagged byte image a diveN.bin
    // holds, then run it through the one canonical row writer (the same call the
    // handset's `LOG EXPORT` and the web's `/dive/n.csv` make on the board).
    static uint8_t packed[kMaxSnaps * logfmt::SNAPSHOT_SIZE +
                          kMaxEvents * logfmt::EVENT_SIZE];
    static logfmt::ImageRecord recs[kMaxSnaps + kMaxEvents];
    int nRec = 0, used = 0;
    for (int i = 0; i < nSnap; i++) {
        logfmt::packSnapshot(snaps[i], packed + used);
        recs[nRec++] = {logfmt::REC_SNAPSHOT, packed + used};
        used += logfmt::SNAPSHOT_SIZE;
    }
    for (int i = 0; i < nEv; i++) {
        logfmt::packEvent(evs[i], packed + used);
        recs[nRec++] = {logfmt::REC_EVENT, packed + used};
        used += logfmt::EVENT_SIZE;
    }
    static uint8_t img[32 * 1024];
    const size_t len =
        logfmt::packImage(img, sizeof(img), 0, 1, recs, (size_t)nRec, &meta);
    if (len == 0) FAIL("dive %d: could not pack the dive image (%d records)", id, nRec);

    static char csv[160 * 1024];
    const size_t n = logfmt::toCsvImage(img, len, csv, sizeof(csv));
    if (n == 0) FAIL("dive %d: CSV export overflowed (%d rows)", id, nRec);
}

}  // namespace

// Batched so one bad dive names itself without losing the rest of the soak.
void test_soak_dives_01_13() { for (int i = 1; i <= 13; i++) runDive(i); }
void test_soak_dives_14_25() { for (int i = 14; i <= 25; i++) runDive(i); }
void test_soak_dives_26_38() { for (int i = 26; i <= 38; i++) runDive(i); }
void test_soak_dives_39_50() { for (int i = 39; i <= 50; i++) runDive(i); }

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_soak_dives_01_13);
    RUN_TEST(test_soak_dives_14_25);
    RUN_TEST(test_soak_dives_26_38);
    RUN_TEST(test_soak_dives_39_50);
    return UNITY_END();
}
