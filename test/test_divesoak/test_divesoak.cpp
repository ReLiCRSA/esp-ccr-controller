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
// this suite fail. What it still cannot reach: the ADC staleness gate and the
// stuck-open hysteresis (the gas model never makes a CELL sample stale or
// dither) — test_loop, test_depth and test_injector own those. What it can reach
// since 2026-11-25: the depth leg. sim::World::DepthFault stops the bus that
// feeds the depth filter, so test_depth_bus_dies_and_returns_mid_dive owns the
// frozen setpoint band and "an unmeasured depth never becomes a published
// depth" end to end, through the real gas model. One limit worth stating: the
// sim's dead bus holds its last sample the way the real driver does, so a chain
// that ignores the validity flag is not caught by the depth asserts alone (the
// numbers are the same either way) — the dive therefore checks its own premise,
// and test_depth owns the filter's hold-last-good rule against a moving input.
// The sim's depth is also noise-free, so spike rejection stays test_depth's.
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
#include "util/numfmt.h"       // the rig must store values the way the firmware does
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
// F_DEPTH_BUS is last (appended): fault kinds are named in BENCH.md evidence,
// so the existing order is not renumberable.
enum FaultKind : uint8_t { F_NONE, F_DEAD, F_SATURATE, F_STUCK_LOW, F_STUCK_HIGH,
                           F_SLOW, F_VALVE, F_FLUSH, F_TWIN_LIE, F_TWIN_DRIFT,
                           F_DEPTH_BUS };

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
        case F_DEPTH_BUS: return "depth bus dead (both chips, one Wire1)";
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

// Did the machine know about this condition, whatever volume it was reporting it
// at? This is the chain's own LOG view (alarms::Result::logBits): the live word
// plus a CRITICAL the diver has ACKed into silence. The suite used to rebuild that
// set by hand out of ackSuppressed because the firmware had no word for it; now
// that the firmware has one, the suite reads it — one vocabulary, and the hand-
// built version could not drift away from what the dive file gets.
uint16_t knownAlarms(const Rig& rig) { return rig.alarmLogBits(); }

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
        case F_DEPTH_BUS:
            // Both depth channels at once — the shared-Wire1 failure. Only the
            // measurement dies: the world keeps its real depth and keeps
            // burning SAC at the real ambient pressure, and the rig keeps
            // feeding the last reading the bus delivered with the not-believed
            // flag set (sim/rig.cpp). That gap is what is under test.
            w.depthFault = DepthFault::BUS_DEAD;
            break;
        default: break;
    }
    f.injected = true;
}

// On-window faults: the measurement comes back. A depth bus that stays dead for
// the whole dive only tests half of the rule — the other half is that the chain
// resumes BELIEVING samples once they are real again, instead of latching into
// "depth is a number we keep" (test_depth_bus_dies_and_returns_mid_dive).
void clearFault(Rig& rig, const Fault& f) {
    switch (f.kind) {
        case F_DEPTH_BUS: rig.world().depthFault = DepthFault::NONE; break;
        default: break;  // cell lies and a stuck valve are not self-clearing
    }
}

// Did the machine see this fault? Detection has to come from the chain itself;
// a cell being dropped from the vote counts as the same news as the drift
// alarm, because that is what the diver is told about.
bool faultSignalSeen(const Rig& rig, const Fault& f, uint16_t mask) {
    if (mask == 0) return true;
    if (knownAlarms(rig) & mask) return true;
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
        const uint16_t seen = knownAlarms(rig);
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
        // The row carries the chain's LOG view, which is what main.cpp writes
        // (sn.alarmBits = st.alarms.logBits): ORed across the row here because the
        // rig samples once per tick and wants "anything true in this window", the
        // stricter of the two. Never the live word: an ACK would delete a fact the
        // row is being read for.
        snapAlarm |= rig.alarmLogBits();
        if (tS >= snapClock && nSnap < kMaxSnaps) {
            snapClock = tS + 10.0f;
            logfmt::Snapshot& s = snaps[nSnap++];
            s.tSec = (uint32_t)tS;
            // numfmt::scaled, not a cast: until 2026-11-24 the rig mirrored the
            // firmware's truncating casts line for line, which is precisely why the
            // soak suite could not see the defect. A rig that re-implements the code
            // under test witnesses nothing.
            s.depthCm = static_cast<int32_t>(numfmt::scaled(dv.depth, 2));
            s.votedX1000 = static_cast<int16_t>(numfmt::scaled(rig.vote().votedPpo2, 3));
            for (int c = 0; c < 3; c++)
                s.cellX1000[c] = static_cast<int16_t>(numfmt::scaled(rig.cellStatus((uint8_t)c).last, 3));
            s.setpointX1000 = static_cast<int16_t>(numfmt::scaled(sp, 3));
            s.mode = rig.autoMode() ? 1 : 0;
            s.alarmBits = snapAlarm;  // anything active since the previous row
            s.cnsPct = (uint8_t)(rig.cnsPct() > 255.0 ? 255.0 : rig.cnsPct());
            s.ceilingCm = static_cast<int16_t>(numfmt::scaled(rig.deco().ceilingM(), 2));
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
    meta.maxDepthCm = static_cast<int32_t>(numfmt::scaled(dv.depthM[0], 2));
    meta.minVotedX1000 = static_cast<int16_t>(numfmt::scaled(rig.minVotedSeen, 3));
    meta.maxVotedX1000 = static_cast<int16_t>(numfmt::scaled(rig.maxVotedSeen, 3));
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
        logfmt::packImage(img, sizeof(img), 0, 1, (uint32_t)id, recs, (size_t)nRec, &meta);
    if (len == 0) FAIL("dive %d: could not pack the dive image (%d records)", id, nRec);

    static char csv[160 * 1024];
    const size_t n = logfmt::toCsvImage(img, len, csv, sizeof(csv));
    if (n == 0) FAIL("dive %d: CSV export overflowed (%d rows)", id, nRec);
}

// ---------------------------------------------------------------------------
// The depth bus dies mid-dive and comes back (findings C1 + C2, end to end)
//
// Why this dive exists: this suite's own header used to admit it could not
// reach the depth filter, because the world had no way to stop MEASURING depth.
// sim::World::DepthFault::BUS_DEAD is that way, and it is the failure the
// hardware actually has — MS5837 and LPS28DFW share one I2C port
// (include/config/pins.h), so a wet harness takes both depth channels out
// together, which is exactly the "both failed" state sensors/depth_manager.h
// budgets for. Physics does NOT follow the fault: the loop keeps burning SAC at
// the real ambient pressure while the controller is blind about where it is.
//
// The window deliberately straddles an ASCENT, and that is the load-bearing
// choice. A dead bus keeps its driver's last pressure (Ms5837::pressureBar_ is
// written only on a good conversion — src/sensors/ms5837.cpp:146 — and no fault
// or reset path clears it, so g_depthM holds too), so with the diver sitting
// still, a chain that BELIEVES the frozen sample and a chain that HOLDS it see
// the same number and no assert can tell them apart. With the diver climbing,
// the two separate: the honest band would step down as the rig rises and the
// chain's must not. The dive therefore checks its own premise (the rig really
// moved while blind) before it claims anything about the hold — a scenario that
// silently stops testing its premise is how a regression comes back green.
//
// Why a named scenario and not another pool draw: pool[] is indexed by the
// per-dive RNG, so adding a kind re-rolls all fifty seeded dives — and those
// dive numbers are cited as evidence in PLAN.md, BENCH.md and in code comments
// ("the 50-dive soak, dive 4"). Re-rolling them is a decision for the
// maintainer, not a side effect of adding a fault. F_DEPTH_BUS is wired into
// the soak's fault vocabulary here (injectFault / clearFault / faultName /
// detection grading) so that when the decision is made it is one line in pool[].
//
// Graded on, every control cycle through the window:
//   * the chain's depth stays a HELD measurement (within 0.5 m of last-good)
//     while the rig itself climbs ~13 m under it — never a surface the machine
//     invented. This is the exact value main.cpp turns into maxDepthCm and the
//     30 s grid row's depthCm, so "no fabricated 0 m reaches the dive file" is
//     asserted at its source;
//   * the setpoint never steps down a rung, let alone to the surface rung, even
//     though the rig's true depth says it should. Note which direction the hold
//     leaves the loop in: a deep setpoint above a shallower reality, i.e. richer
//     than the honest plan. That is inside the toxicity fences, which is why the
//     per-cell high veto and the 2.20 bar envelope are asserted here too;
//   * the injector keeps regulating to the held band;
//   * BIT_SENSOR_STALE is visible the whole window (announced within its
//     deadline, by the suite's own detection-grading idiom);
//   * BIT_SENSOR_FAIL never appears: the cells are honest throughout, and a
//     chain that reported a lost vote here would be conflating "depth
//     unmeasured" with "no ppO2 measurement", which is the C2 boundary;
//   * the exposure picture keeps integrating (follow-up #1): the CNS clock
//     accrues through the window — ppO2 comes from the cells, not the depth bus
//     — and the tissue model keeps counting the held depth. Direction is
//     asserted against a TWIN: a second rig, same options, same diver, same true
//     depth at every tick, but an honest bus. Up to the moment the fault opens
//     the two are deterministically identical (same dive, same world noise
//     seed), so whatever separates them at the window's end IS the fault. It
//     must separate in the conservative direction: the blind rig's tissues are
//     the loaded ones and the ceiling it shows is the deeper one, because it
//     kept integrating the last measured depth while the real diver climbed out
//     of it. Freezing (the old behavior) sat between the two and quietly
//     forgot both the loading and the off-gas the diver actually did. The
//     witness bundles the held setpoint's effect on ppN2 as well — it asserts
//     the direction, not a decomposition of it; test_loop's
//     test_blind_depth_window_still_clocks_cns_and_loads_tissues is the one
//     that pins the magnitudes;
//   * after the bus returns, the chain resumes believing samples within the
//     filter's own settle bound, and the band is free to move again at the end
//     of the dive (the freeze is a held input, not a latched mode);
//   * and the window is identifiable in the dive file afterwards, with no wire
//     or format change: every 30 s grid row wholly inside it carries
//     BIT_SENSOR_STALE in alarmBits (main.cpp writes the row from the LOG view,
//     alarms::Result::logBits) and its ceiling/decoTotal/NDL columns — written
//     from the same deco picture the window just integrated — actually moved. A
//     blind window that integrated nothing would leave rows that look healthy and
//     unchanged, which is how an under-recorded deco debt hides. The diver ACKs
//     the alarm 8 s into the window, so this grades the two vocabularies against
//     each other at the same instant: the LIVE word must lose BIT_SENSOR_STALE to
//     the ACK (the handset went quiet — otherwise the marker assert proves
//     nothing, because nothing separates the views) while the file's rows keep it,
//     and the log view must stop claiming it once the bus recovers (a fact about
//     now, not a latch). That split is finding #5.
// Mirrored from runDive's invariants: setpoint escape, the low-loop rule with
// its sensor-honest escape hatch, the 0.12/2.20 bar hard envelope, the per-cell
// high veto, and the latched-valve rule. Deliberately not mirrored: the
// high-unalarmed pair, which grades cell-LIE detection — a fault this dive does
// not inject, and asserting it here would be asserting an unrelated path.
// The deco engine no longer freezes while the bus is blind: both halves advance
// through the window (loop.cpp step 8), which is what the CNS and twin asserts
// below are watching.
// ---------------------------------------------------------------------------
void runDepthBusDive() {
    const int id = 9001;                // the scenario's own number, never a soak dive
    constexpr float kBottomM = 32.0f;   // the SP3 band: 1.00 bar, deep enough to matter
    constexpr float kRiseToM = 12.0f;   // the honest band up there is SP2 0.70, not 1.00
    constexpr float kFaultAtS = 420.0f;  // opens ~7 s before the climb starts, past CellProof
    constexpr float kWindowS = 90.0f;   // 900 control ticks: long enough to matter
    // Band floor, not equality: CellProof's upward override (1.10) is not a walk
    // and must not fail this. Downward is: 0.70 is one rung lost and 0.40 is the
    // surface, which is the C1 failure.
    constexpr float kSpFloorBar = 0.999f;
    // The loop is held near 1.0 bar by a setpoint the rig's real depth no longer
    // supports, so it still falls toward it and still owes pulses: around 20 m it
    // burns 1.2 l/min x 3.0 bar ~= 3.6 l-atm/min out of 8 l, about 0.45 bar/min,
    // so a pulse is owed every ~4 s — roughly 20 in this window. Ten is the
    // "injection clearly still running" bar; a setpoint that walked to 0.40 bar
    // would have injected almost none, because the loop already sits at 1.0.
    constexpr int kMinPulsesInWindow = 10;
    // The premise of the whole dive: the rig must actually have moved under the
    // held reading (see the comment above). 5 m is 34 s of climb at 0.15 m/s and
    // well past the 0.5 m the chain is allowed to hold within.
    constexpr float kPremiseRiseM = 5.0f;
    const float trackAfterS = kFaultAtS + kWindowS + 3.0f;  // filter bound is ~0.9 s
    constexpr float kTrackTolM = 1.2f;  // 9 ticks of settle + 0.15 m/s of real climb

    RigOpts o = soakOpts();
    Rig rig(o);
    prediveHealthy(rig);
    if (!rig.predivePassed()) FAIL("dive %d: pre-dive failed on a rig with no fitted faults", id);
    if (!rig.requestAuto(true)) FAIL("dive %d: AUTO refused after a passed pre-dive", id);

    rig.startDive();
    rig.markEnvelope();

    // The twin: same options, same diver, the same true depth at every tick, but
    // an honest depth bus. Until the fault opens the two are deterministically
    // the same dive, so the difference between them at the window's end is the
    // fault and nothing else — which is how the exposure asserts below can
    // assert a DIRECTION (the blind rig must be the conservative one) without
    // hard-coding a deco number that a GF or band change would move. The diver
    // follows the FAULTED rig's ceiling (that is what a diver looking at this
    // machine's display would do), which also keeps both rigs at the same true
    // depth, so the tissue comparison is apples to apples. The twin is a
    // reference, not a subject: the suite's safety invariants are graded on the
    // faulted rig, and the twin's behavior is the healthy-dive path the other
    // fifty dives already grade.
    Rig twin(o);
    prediveHealthy(twin);
    if (!twin.predivePassed()) FAIL("dive %d: the twin's pre-dive failed", id);
    if (!twin.requestAuto(true)) FAIL("dive %d: the twin refused AUTO", id);
    twin.startDive();

    // Bottom, then a climb whose middle is inside the blind window: two
    // waypoints, driven by the soak's own ceiling-aware, rate-limited diver.
    Diver dv;
    dv.depthM[0] = kBottomM;
    dv.holdS[0] = 300.0f;
    dv.depthM[1] = kRiseToM;
    dv.holdS[1] = 300.0f;
    dv.nWp = 2;
    dv.holdLeft = dv.holdS[0];

    Fault f;
    f.kind = F_DEPTH_BUS;
    f.atS = kFaultAtS;
    f.detectMask = (uint16_t)(1u << alarms::BIT_SENSOR_STALE);
    f.detectDeadlineS = 1.0f;  // one tick: the bus health flag is not something to infer
    const float clearAtS = f.atS + kWindowS;

    // The CNS the window is owed: the loop is regulated to the held 1.00 bar
    // band throughout, so the NOAA 1.00 row (300 min) sets the rate —
    // 100 x 1.5 min / 300 = 0.5 %, and the CellProof override can push it a bit
    // higher. Anything near zero is the frozen clock this follow-up removed.
    constexpr float kMinCnsGainPct = 0.25f;

    const float dtS = Rig::kTickMs / 1000.0f;
    const float ackDelayS = 8.0f;  // a human moment to notice and ACK
    bool inWindow = false, windowClosed = false, trackChecked = false;
    float heldDepthM = 0.0f, minSp = 9.0f, minChainM = 999.0f, maxRiseM = 0.0f;
    float cnsAtOpen = 0.0f, cnsGainInWindow = 0.0f;
    double fastTissueOpen = 0.0, twinFastTissueOpen = 0.0;
    double fastTissueClose = 0.0, twinFastTissueClose = 0.0;
    double ceilAtClose = 0.0, twinCeilClose = 0.0;
    // The 30 s dive-file row, tracked the way the firmware writes it.
    float rowClockS = 0.0f, rowDecoOpen = 0.0f, rowDecoClose = 0.0f;
    float rowNdlOpen = 0.0f, rowNdlClose = 0.0f;
    float rowCeilLastBlind = 0.0f, ceilAtWindowOpen = 0.0f;
    int rowsBlind = 0, blindCyclesInRow = 0, rowsBlindMarker = 0;
    // The live-vs-log split, witnessed cycle by cycle. `ackSilencedCycles` counts
    // the cycles where the diver's ACK had taken BIT_SENSOR_STALE off the live
    // word while the log view still carried it: the ACK working (live) and the
    // dive file not forgetting (log) at the same instant. `logStaleAfterRecovery`
    // catches the opposite error — a log view that outlives the condition it
    // reports, which would make it a latch rather than a fact.
    int ackSilencedCycles = 0;
    bool logStaleAfterRecovery = false;
    uint32_t pulsesAtFault = 0, pulsesInWindow = 0;
    bool staleMissing = false, sensorFailSeen = false;
    float lowUnalarmedS = 0.0f, alarmSinceS = -1.0f;
    uint16_t alarmUnion = 0;
    bool finished = false;
    float tS = 0.0f;

    for (; tS < 2400.0f && !finished; tS += dtS) {
        // ---- open, then close, the window --------------------------------
        if (!f.injected && tS >= f.atS) {
            injectFault(rig, f);
            inWindow = true;
            heldDepthM = rig.loop().depthSmoothedM();
            pulsesAtFault = rig.pulseCount();
            if (heldDepthM < kBottomM - 1.0f)
                FAIL("dive %d: the chain reads %.1f m at the bottom before the fault (true "
                     "%.1f m) — the scenario is measuring the wrong thing",
                     id, heldDepthM, dv.depth);
            // The twins must still be the same dive at this instant: if they are
            // not, the comparison at the window's end proves nothing.
            if (std::fabs(rig.deco().tissueBar(0) - twin.deco().tissueBar(0)) > 0.02)
                FAIL("dive %d: twin (%.2f bar) and faulted rig (%.2f bar) had different tissues "
                     "before the fault was injected — the comparison below is not a control",
                     id, twin.deco().tissueBar(0), rig.deco().tissueBar(0));
            cnsAtOpen = rig.cnsPct();
            fastTissueOpen = rig.deco().tissueBar(0);
            twinFastTissueOpen = twin.deco().tissueBar(0);
            // The deco columns of the next dive-file row, computed the way
            // main.cpp computes them: from the depth the CHAIN believes (the
            // held one during the window), not the world's true one.
            const double heldBar = rig.deco().depthToBar(rig.loop().depthSmoothedM());
            rowDecoOpen = (float)rig.deco().schedule(heldBar).totalTimeMin;
            rowNdlOpen = (float)rig.deco().ndlMinutes(heldBar, rig.vote().votedPpo2);
            ceilAtWindowOpen = (float)rig.deco().ceilingM();
        }
        if (inWindow && tS >= clearAtS) {
            clearFault(rig, f);
            inWindow = false;
            windowClosed = true;
            pulsesInWindow = rig.pulseCount() - pulsesAtFault;
            // The premise, checked at the moment it could still have been met:
            // if the rig never moved under the held reading, believing and
            // holding are the same number and this dive proves nothing.
            if (maxRiseM < kPremiseRiseM)
                FAIL("dive %d: the rig moved only %.1f m while the bus was blind (held %.1f "
                     "m, true %.1f m) — this dive cannot tell a held depth from a believed "
                     "one; move the window (profile changed?)",
                     id, maxRiseM, heldDepthM, dv.depth);

            // ---- the exposure picture, at the moment the window closes ----
            // CNS: ppO2 is a cell measurement, so the oxygen clock must have run
            // the whole window. The old chain froze it here (exactly 0 gained).
            cnsGainInWindow = rig.cnsPct() - cnsAtOpen;
            if (cnsGainInWindow < kMinCnsGainPct)
                FAIL("dive %d: the blind %0.0f s window gained %.2f %% CNS at a held %.2f bar "
                     "setpoint (the NOAA 1.00 row owes ~0.5 %%): ppO2 exposure stopped being "
                     "counted because the DEPTH bus died",
                     id, kWindowS, cnsGainInWindow, minSp);
            // Tissues: the model keeps integrating the held last-good depth, so
            // its fast compartment is still loading toward 32 m while the real
            // diver has climbed ~12 m out of that band. The honest twin, same
            // dive, same diver, is off-gassing instead. The blind rig must be
            // the deeper one — over-recording debt asks for MORE deco, which is
            // the fenced direction; forgetting it asks for less.
            fastTissueClose = rig.deco().tissueBar(0);
            twinFastTissueClose = twin.deco().tissueBar(0);
            const double fastDelta = fastTissueClose - fastTissueOpen;
            const double twinFastDelta = twinFastTissueClose - twinFastTissueOpen;
            if (fastDelta + 0.1 < twinFastDelta)
                FAIL("dive %d: across the blind window the faulted rig's fast compartment moved "
                     "%+.3f bar against the honest twin's %+.3f — the held-depth integration "
                     "under-recorded the diver's debt (last measured depth %.1f m, true %.1f m)",
                     id, fastDelta, twinFastDelta, heldDepthM, dv.depth);
            // And the same in the number the diver is shown: the ceiling this
            // machine demands must not be shallower than the honest one's.
            ceilAtClose = rig.deco().ceilingM();
            twinCeilClose = twin.deco().ceilingM();
            const double heldBarClose = rig.deco().depthToBar(rig.loop().depthSmoothedM());
            rowDecoClose = (float)rig.deco().schedule(heldBarClose).totalTimeMin;
            rowNdlClose = (float)rig.deco().ndlMinutes(heldBarClose, rig.vote().votedPpo2);
            if (ceilAtClose + 0.25 < twinCeilClose)
                FAIL("dive %d: the blind rig shows a %.1f m ceiling against the twin's %.1f m "
                     "— the window's debt never reached the deco picture the diver reads",
                     id, ceilAtClose, twinCeilClose);
            if (fastTissueClose <= fastTissueOpen)
                FAIL("dive %d: no tissue moved during the %0.0f s blind window (fast compartment "
                     "%.2f -> %.2f bar): deco integration froze with the depth bus",
                     id, kWindowS, fastTissueOpen, fastTissueClose);
        }

        if (dv.step(dtS, rig.deco(), tS)) { finished = true; break; }
        rig.world().setDepthM(dv.depth);
        rig.tick();
        // The twin rides the same diver and the same true depth, with an honest
        // bus: its world burns SAC at the same ambient pressure and its chain
        // believes every sample it is given.
        twin.world().setDepthM(dv.depth);
        twin.tick();

        const uint16_t bits = rig.alarmBits();
        const uint16_t seen = knownAlarms(rig);
        alarmUnion |= bits;
        if (f.injected && f.detectSeenS < 0.0f && faultSignalSeen(rig, f, f.detectMask))
            f.detectSeenS = tS;

        // ---- the live/log split, at one instant ----------------------------
        // The simulated diver ACKs (below), so during a long blind window there
        // are cycles where the live word no longer carries BIT_SENSOR_STALE — the
        // ACK did its job, the handset went quiet — while the view the dive file
        // is written from still does. Both halves of that sentence have to be
        // witnessed, or a change to either view passes unnoticed.
        if (inWindow && !(bits & (1u << alarms::BIT_SENSOR_STALE)) &&
            (seen & (1u << alarms::BIT_SENSOR_STALE)))
            ackSilencedCycles++;
        // And the log view is a fact, not a latch: once the depth bus is back and
        // the condition has genuinely recovered, it must stop claiming STALE too.
        if (windowClosed && tS >= clearAtS + 5.0f && (seen & (1u << alarms::BIT_SENSOR_STALE)))
            logStaleAfterRecovery = true;

        const float sp = rig.spTarget();
        const float chainM = rig.loop().depthSmoothedM();
        const float pTrue = rig.world().ppo2True();

        // Setpoint escape: the suite-wide bound first, whatever the depth leg does.
        if (sp < 0.39f || sp > 1.31f)
            FAIL("dive %d: setpoint escaped to %.2f bar at t=%.0fs depth=%.1fm",
                 id, sp, tS, dv.depth);

        if (inWindow) {
            // A held measurement, not an invented surface. chainM is what
            // main.cpp feeds maxDepthCm and the 30 s grid row: if it were 0 m
            // here, the dive file would say the diver was on the surface.
            if (chainM < heldDepthM - 0.5f)
                FAIL("dive %d: depth readout fell to %.1f m with the bus blind (last good "
                     "%.1f m, true depth %.1f m) at t=%.0fs — an unmeasured depth is "
                     "being believed", id, chainM, heldDepthM, dv.depth, tS);
            if (chainM < minChainM) minChainM = chainM;
            if (sp < minSp) minSp = sp;
            if (sp < kSpFloorBar)
                FAIL("dive %d: dead depth bus walked the setpoint to %.2f bar at t=%.0fs "
                     "(held band 1.00, last good depth %.1f m, true %.1f m): the surface "
                     "rung was chased", id, sp, tS, heldDepthM, dv.depth);
            if (!(seen & (1u << alarms::BIT_SENSOR_STALE))) staleMissing = true;
            if (seen & (1u << alarms::BIT_SENSOR_FAIL)) sensorFailSeen = true;
            if (rig.vote().mode == voting::Mode::FAILED)
                FAIL("dive %d: the vote failed on a dive whose only fault was depth "
                     "(t=%.0fs alarms=0x%03x)", id, tS, alarmUnion);
            // The premise, watched from the diving side: the rig is climbing out
            // of the band the chain is holding. If this never happens, the
            // window is not straddling a depth change and the assert above is
            // not doing the job it looks like it does.
            if (heldDepthM - dv.depth > maxRiseM) maxRiseM = heldDepthM - dv.depth;
        }

        // Post-recovery: the chain must be back on the world's depth within the
        // filter's own settle bound (5 medians + 4 boxcar = 9 ticks), plus the
        // ground the climb covers in the 3 s since the window closed. A hold
        // that outlives the fault is a depth frozen forever.
        if (windowClosed && !trackChecked && tS >= trackAfterS) {
            trackChecked = true;
            if (std::fabs(chainM - rig.world().depthM()) > kTrackTolM)
                FAIL("dive %d: with the bus back the chain still reads %.1f m while the rig "
                     "is at %.1f m — hold-last-good outlived the fault",
                     id, chainM, rig.world().depthM());
        }

        // ---- the suite-wide safety invariants, through the window ---------
        // The low-loop rule carries the sensor-honest escape hatch: it exists for
        // a machine that is shouting it cannot be trusted. A depth fault must not
        // NEED it — the loop stays regulated on the held band — so the assert is
        // here to prove the window is inert, and it is the second net if a setpoint
        // walk ever gets far enough to starve the loop (the setpoint assert above
        // fires first).
        if (pTrue < 0.18f) {
            lowUnalarmedS += dtS;
            const bool sensorHonest = seen & ((1u << alarms::BIT_CELL_DRIFT) |
                                              (1u << alarms::BIT_SENSOR_FAIL) |
                                              (1u << alarms::BIT_SENSOR_STALE));
            if (lowUnalarmedS > 3.0f &&
                !(seen & ((1u << alarms::BIT_LOW_PPO2) | (1u << alarms::BIT_HYPOXIA))) &&
                !sensorHonest)
                FAIL("dive %d: loop at %.2f bar for %.1f s with no low/hypoxia alarm and "
                     "no sensor warning (t=%.0fs depth=%.1fm alarms=0x%03x)",
                     id, pTrue, lowUnalarmedS, tS, dv.depth, alarmUnion);
        } else lowUnalarmedS = 0.0f;
        if (pTrue < 0.12f && dv.depth > 3.0f)
            FAIL("dive %d: loop fell to %.2f bar at %.1f m (t=%.0fs alarms=0x%03x)",
                 id, pTrue, tS, dv.depth, alarmUnion);
        if (pTrue > 2.20f)
            FAIL("dive %d: loop reached %.2f bar (toxic) at t=%.0fs depth=%.1fm alarms=0x%03x",
                 id, pTrue, tS, dv.depth, alarmUnion);
        float maxCell = 0.0f;
        for (int c = 0; c < 3; c++)
            if (rig.cellStatus((uint8_t)c).last > maxCell) maxCell = rig.cellStatus((uint8_t)c).last;
        if (maxCell >= kHighWall && rig.injOut().pulseStarted)
            FAIL("dive %d: pulse started while a cell read %.2f bar (wall %.2f) at t=%.0fs "
                 "vote=%.2f alarms=0x%03x", id, maxCell, kHighWall, tS, rig.vote().votedPpo2,
                 alarmUnion);
        if (rig.injOut().stuckOpenLatched && rig.loop().solenoid())
            FAIL("dive %d: valve still commanded while latched stuck-open (t=%.0fs)", id, tS);

        // ---- the simulated diver -----------------------------------------
        if (bits && alarmSinceS < 0.0f) alarmSinceS = tS;
        if (!bits) alarmSinceS = -1.0f;
        if (alarmSinceS >= 0.0f && tS - alarmSinceS >= ackDelayS) {
            rig.ackAlarms();
            alarmSinceS = -1.0f;
        }

        // ---- the 30 s dive-file row, filled the way main.cpp fills it ------
        // The row's alarmBits is written from the LOG view at the row's own instant
        // (main.cpp: sn.alarmBits = st.alarms.logBits), so a row wholly inside this
        // window must carry BIT_SENSOR_STALE even though the diver ACKed the alarm
        // 8 s into it. That instant is read directly, not an OR across the row:
        // main.cpp writes one value per row, and an OR would let a row inherit a
        // marker from cycles before the ACK and hide a file that goes blind to the
        // condition the moment the diver asks for quiet.
        if (inWindow) blindCyclesInRow++;
        if (tS >= rowClockS) {
            // Only a row that was blind for most of its 30 s says anything: a row
            // clipped by one cycle at the edge of the window would be graded on a
            // cycle whose alarm had barely had time to arrive.
            // Grade a row only when it was blind for most of its 30 s AND the row
            // boundary itself fell inside the window: a boundary at or after the
            // recovery lets the first healthy cycles move the numbers and hides a
            // window in which nothing integrated at all.
            if (blindCyclesInRow >= 150 && inWindow) {
                rowsBlind++;
                if (rig.alarmLogBits() & (uint16_t)(1u << alarms::BIT_SENSOR_STALE))
                    rowsBlindMarker++;
                rowCeilLastBlind = (float)rig.deco().ceilingM();
            }
            rowClockS += 30.0f;
            blindCyclesInRow = 0;
        }
    }
    rig.endDive();
    twin.endDive();

    // ---- grading ----------------------------------------------------------
    if (!f.injected) FAIL("dive %d: the depth-bus fault was never injected", id);
    if (!windowClosed) FAIL("dive %d: the depth-bus window never closed (t=%.0fs)", id, tS);
    if (!finished) FAIL("dive %d: the dive had not finished by t=%.0fs", id, tS);
    if (f.detectSeenS < 0.0f)
        FAIL("dive %d: a dead depth bus raised no sensor alarm at all (alarms=0x%03x)",
             id, alarmUnion);
    if (f.detectSeenS - f.atS > f.detectDeadlineS)
        FAIL("dive %d: the dead depth bus took %.0fs to announce (deadline %.0fs)",
             id, f.detectSeenS - f.atS, f.detectDeadlineS);
    if (staleMissing)
        FAIL("dive %d: part of the blind window ran with no sensor alarm known to the chain — "
             "the view the dive file is written from (alarms=0x%03x)", id, alarmUnion);
    if (sensorFailSeen)
        FAIL("dive %d: honest cells reported as a lost vote: depth staleness conflated "
             "with no-vote (alarms=0x%03x)", id, alarmUnion);
    if (pulsesInWindow < kMinPulsesInWindow)
        FAIL("dive %d: only %u pulses in the %u s blind window (lowest setpoint %.2f bar): "
             "the injector stopped regulating to the held band",
             id, (unsigned)pulsesInWindow, (unsigned)kWindowS, minSp);
    if (minChainM < 20.0f)
        FAIL("dive %d: the chain published %.1f m while blind at %.0f m — a fabricated "
             "surface reached the dive-file inputs", id, minChainM, kBottomM);
    if (!trackChecked) FAIL("dive %d: the dive ended before the post-recovery check ran", id);
    // The window must be identifiable post-dive, with no format change: two 30 s
    // rows of a 90 s window are wholly inside it (their boundaries fall between
    // the open and the close), every one of them has to carry the sensor alarm in
    // the view the file is written from — across the ACK — and the deco columns
    // written beside it have to have moved.
    if (rowsBlind < 2)
        FAIL("dive %d: only %d dive-file rows sat wholly inside the %0.0f s window — the profile "
             "moved on, the witness below proves nothing", id, rowsBlind, kWindowS);
    // ---- finding #5, now a requirement ------------------------------------
    // Every dive-file row wholly inside the window must carry BIT_SENSOR_STALE,
    // even though the diver ACKed the alarm 8 s into the window. This used to be a
    // NOTE, because the firmware wrote the LIVE word into the file and had no log
    // view to write instead: an ACK deleted the marker from the very rows whose
    // ceiling/NDL were computed with no depth measurement. Now the file carries
    // alarms::Result::logBits, so losing the marker is a defect, not a footnote.
    // The row's own instant is what is read, because that is what main.cpp writes.
    // It is deliberately NOT "was STALE live somewhere in the row": the ACK
    // silences the live word 8 s in, so no later row could pass that, and whether
    // the chain announced the fault at all is graded separately, above.
    if (rowsBlindMarker != rowsBlind)
        FAIL("dive %d: %d/%d dive-file rows wholly inside the blind window would be written "
             "without BIT_SENSOR_STALE — the diver's ACK deleted a fact about the instrument "
             "state (the file is written from logBits, not the live word)",
             id, rowsBlind - rowsBlindMarker, rowsBlind);
    // The other side of the same instant: the ACK has to have actually silenced the
    // LIVE word, or the assert above is not distinguishing two views at all — it
    // would pass on a build where ACK silences nothing. This is the half that keeps
    // the handset's `ACK` meaningful, so it is graded as hard as the marker.
    if (ackSilencedCycles == 0)
        FAIL("dive %d: across %d blind rows the diver's ACK never took BIT_SENSOR_STALE off "
             "the live word — nothing here separates the live view from the log view, so the "
             "marker assert above proves nothing (alarms=0x%03x)", id, rowsBlind, alarmUnion);
    // And the log view reports conditions, it does not latch them: once the bus is
    // back the marker has to stop, or the file would claim blindness that ended.
    if (logStaleAfterRecovery)
        FAIL("dive %d: 5 s after the depth bus recovered, the dive-file view still claimed "
             "BIT_SENSOR_STALE — the log view is a fact about now, not a latch that "
             "outlives its condition", id);
    if (!(rowDecoClose > rowDecoOpen || rowNdlClose < rowNdlOpen))
        FAIL("dive %d: across the blind window the deco columns of the dive file did not move "
             "(total %.1f -> %.1f min, NDL %.1f -> %.1f min): the window left no trace the "
             "held-depth integration could be seen in", id, rowDecoOpen, rowDecoClose, rowNdlOpen,
             rowNdlClose);
    if (rowCeilLastBlind <= ceilAtWindowOpen)
        FAIL("dive %d: the ceiling column written into the dive file was flat across the blind "
             "window (%.2f m when it opened, %.2f m on the last blind row): the integration that "
             "ran on the held depth left nothing a post-dive reader could see",
             id, ceilAtWindowOpen, rowCeilLastBlind);
    if (std::fabs(rig.spTarget() - 0.40f) > 0.001f)
        FAIL("dive %d: home at the surface and the setpoint is still %.2f bar — the depth "
             "freeze latched instead of tracking the recovered bus", id, rig.spTarget());

    std::printf("dive %d  depth-bus  %5.0f m  window %3.0f s  held %4.1f m  blind rise %4.1f m  "
                "min chain %4.1f m  min sp %.2f  pulses %3u  CNS %4.1f%% (+%2.1f in window, twin "
                "%4.1f%%)  fast tissue %4.2f vs twin %4.2f  ceil %4.1f vs twin %4.1f m  rows %d "
                "(deco %.1f->%.1f min, NDL %.0f->%.0f)  alarms 0x%03x\n",
                id, kBottomM, kWindowS, heldDepthM, maxRiseM, minChainM, minSp,
                (unsigned)pulsesInWindow, rig.cnsPct(), cnsGainInWindow, twin.cnsPct(),
                fastTissueClose, twinFastTissueClose, ceilAtClose, twinCeilClose, rowsBlind,
                rowDecoOpen, rowDecoClose, rowNdlOpen, rowNdlClose, alarmUnion);
    // The vocabulary split, on the record beside the numbers it protects. The
    // alarms column above is the LIVE union (0x080 for this dive, STALE included,
    // because the alarm is raised before the ACK and after recovery); what this
    // line says is that the ACK cost the file nothing.
    std::printf("dive %d  alarm-view  %d/%d blind dive-file rows keep BIT_SENSOR_STALE across "
                "the ACK  (live word silent for %d cycles, log view clear after recovery)\n",
                id, rowsBlindMarker, rowsBlind, ackSilencedCycles);
}

}  // namespace

// Batched so one bad dive names itself without losing the rest of the soak.
void test_soak_dives_01_13() { for (int i = 1; i <= 13; i++) runDive(i); }
void test_soak_dives_14_25() { for (int i = 14; i <= 25; i++) runDive(i); }
void test_soak_dives_26_38() { for (int i = 26; i <= 38; i++) runDive(i); }
void test_soak_dives_39_50() { for (int i = 39; i <= 50; i++) runDive(i); }

// The depth leg, on a named dive rather than a seeded draw: see the comment at
// runDepthBusDive for why the pool is left alone.
void test_depth_bus_dies_and_returns_mid_dive() { runDepthBusDive(); }

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_soak_dives_01_13);
    RUN_TEST(test_soak_dives_14_25);
    RUN_TEST(test_soak_dives_26_38);
    RUN_TEST(test_soak_dives_39_50);
    RUN_TEST(test_depth_bus_dies_and_returns_mid_dive);
    return UNITY_END();
}
