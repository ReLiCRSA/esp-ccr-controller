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
#include "log/rowfill.h"      // the firmware's own row fill: the soak calls it, it does not mirror it
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
        // The row carries the chain's LOG view, which is what the fill writes
        // (logfmt::RowInputs::alarmLogBits): ORed across the row here because the
        // rig samples once per tick and wants "anything true in this window", the
        // stricter of the two. Never the live word: an ACK would delete a fact the
        // row is being read for.
        snapAlarm |= rig.alarmLogBits();
        if (tS >= snapClock && nSnap < kMaxSnaps) {
            snapClock = tS + 10.0f;
            // Not a mirror any more: the rig calls the firmware's own fill
            // (src/log/rowfill.h), the one main.cpp calls. Until now the soak
            // re-typed those conversions — and re-typed them differently, reading
            // the DIVER's depth (dv.depth) and the world's true ambient for the
            // deco columns where the firmware reads the chain's held depth. In a
            // dive with an honest depth bus the two agree closely enough to hide
            // the difference; in a blind one it is the whole point, and a soak row
            // built from the diver's depth could never have caught a firmware that
            // wrote the diver's depth.
            logfmt::RowInputs ri{};
            ri.nowMs = static_cast<uint32_t>(tS * 1000.0f);  // dive-relative
            ri.diveStartMs = 0;
            ri.smoothedDepthM = rig.loop().depthSmoothedM();
            ri.haveVote = rig.loop().haveVote();
            ri.shownPpo2 = rig.loop().shownPpo2();
            for (int c = 0; c < 3; c++)
                ri.cellBar[c] = rig.cellStatus(static_cast<uint8_t>(c)).last;
            ri.setpointBar = sp;
            ri.diveActive = true;
            ri.autoMode = rig.autoMode();
            // The row spans 10 s of simulated time and the rig samples every
            // 100 ms, so the alarm field is ORed across the window: the stricter
            // of the two readings of "what was true at this row". The firmware
            // writes one instant. Never the live word: an ACK would delete a fact
            // the row is being read for.
            ri.alarmLogBits = snapAlarm;
            const long long cnsX = numfmt::scaled(rig.cnsPct(), 0);
            ri.cnsPeakPct = static_cast<uint8_t>(cnsX > 254 ? 254 : cnsX);
            ri.vbat = VBAT_CELLS * 3.90f;
            ri.pulsesNow = rig.pulseCount();
            ri.pulsesPrev = pulsesLastSnap;
            // The published picture, computed exactly the way ControlTask
            // computes it (deco/view.h): the soak mirrors the PUBLICATION,
            // not the engine walk — since N8 the engine walk does not exist.
            ri.hasDecoPicture = true;
            deco::viewFull(ri.deco, rig.loop().deco(),
                           rig.loop().deco().depthToBar(ri.smoothedDepthM),
                           ri.haveVote, ri.shownPpo2);
            logfmt::fillSnapshot(snaps[nSnap++], ri);
            pulsesLastSnap = rig.pulseCount();
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
//     BIT_SENSOR_STALE in alarmBits and its ceiling/NDL/decoTotal columns — the
//     same deco picture the window just integrated — moved. Since 2026-11-25 the
//     row is not re-typed here: the scenario calls logfmt::fillSnapshot
//     (src/log/rowfill.h), the fill main.cpp calls, with the values main.cpp maps,
//     so what these asserts grade are the bytes a diver would download. That also
//     makes the row the right place to grade the vocabulary split: the diver ACKs
//     the alarm 8 s into the window, so the LIVE word must lose BIT_SENSOR_STALE
//     to the ACK (the handset went quiet — otherwise the marker assert proves
//     nothing, because nothing separates the views) while the row keeps it, and
//     the log view must stop claiming it once the bus recovers (a fact about now,
//     not a latch). That split is finding #5.
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
    // The 30 s dive-file row, built by the firmware's own fill every 30 s of the
    // dive: `row` is the row as written, `rowHonest` the last one before the bus
    // went blind, `rowBlind` the last wholly-blind one. All three are logfmt
    // records, so the asserts below grade bytes a diver would download rather than
    // a paraphrase of them.
    logfmt::Snapshot row{}, rowHonest{}, rowBlind{};
    uint32_t rowPulseBase = 0;
    float rowTrueDepthM = 0.0f;
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

        // ---- the 30 s dive-file row, built by the firmware's own fill -------
        // logfmt::fillSnapshot (src/log/rowfill.h) is what main.cpp calls, with
        // the same values main.cpp maps. There is no hand-written row in this
        // scenario any more: what the asserts below grade is the record the
        // controller would have written, conversions and all.
        if (inWindow) blindCyclesInRow++;
        if (tS >= rowClockS) {
            logfmt::RowInputs ri{};
            ri.nowMs = static_cast<uint32_t>(tS * 1000.0f);  // dive-relative
            ri.diveStartMs = 0;
            ri.smoothedDepthM = rig.loop().depthSmoothedM();
            ri.haveVote = rig.loop().haveVote();
            ri.shownPpo2 = rig.loop().shownPpo2();
            for (int c = 0; c < 3; c++)
                ri.cellBar[c] = rig.cellStatus(static_cast<uint8_t>(c)).last;
            ri.setpointBar = rig.spTarget();
            ri.diveActive = true;
            ri.autoMode = rig.autoMode();
            // One instant, exactly like the firmware's grid row: the marker has to
            // survive the ACK at the moment the row is written, not merely have
            // been true somewhere in the 30 s before it.
            ri.alarmLogBits = rig.alarmLogBits();
            const long long cnsX = numfmt::scaled(rig.cnsPct(), 0);
            ri.cnsPeakPct = static_cast<uint8_t>(cnsX > 254 ? 254 : cnsX);
            ri.vbat = VBAT_CELLS * 3.90f;
            ri.pulsesNow = rig.pulseCount();
            ri.pulsesPrev = rowPulseBase;
            // The published picture, computed exactly the way ControlTask
            // computes it (deco/view.h): the soak mirrors the PUBLICATION,
            // not the engine walk — since N8 the engine walk does not exist.
            ri.hasDecoPicture = true;
            deco::viewFull(ri.deco, rig.loop().deco(),
                           rig.loop().deco().depthToBar(ri.smoothedDepthM),
                           ri.haveVote, ri.shownPpo2);
            logfmt::fillSnapshot(row, ri);
            rowPulseBase = ri.pulsesNow;

            // The last row written before the bus went blind: the baseline the
            // blind rows are compared with, produced by the same fill.
            if (!inWindow && !windowClosed) rowHonest = row;

            // Only a row that was blind for most of its 30 s says anything: a row
            // clipped by one cycle at the edge of the window would be graded on a
            // cycle whose alarm had barely had time to arrive.
            // Grade a row only when it was blind for most of its 30 s AND the row
            // boundary itself fell inside the window: a boundary at or after the
            // recovery lets the first healthy cycles move the numbers and hides a
            // window in which nothing integrated at all.
            if (blindCyclesInRow >= 150 && inWindow) {
                rowsBlind++;
                rowBlind = row;
                rowTrueDepthM = dv.depth;  // where the diver actually was at THIS row
                if (row.alarmBits & (uint16_t)(1u << alarms::BIT_SENSOR_STALE))
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
    // ---- the row as a record, not as a paraphrase -------------------------
    // Everything above grades the chain's state; this block grades the BYTES a
    // diver would download, produced by the same fill main.cpp calls. A fill that
    // wrote the diver's depth, or measured the deco columns at it, passes every
    // chain-level assert in this scenario and still lies in the file.
    // (1) The depth column is the chain's HELD depth, and by this row the diver is
    // well over 2 m shallower than the number written beside the marker.
    if (rowBlind.depthCm < static_cast<int32_t>(numfmt::scaled(rowTrueDepthM, 2)) + 200)
        FAIL("dive %d: the blind dive-file row stored %.2f m while the diver was at %.1f m — "
             "the row carries the diver's depth, not the chain's held one, so the STALE marker "
             "beside it describes a depth the file never used",
             id, rowBlind.depthCm / 100.0f, rowTrueDepthM);
    // (2) The window's gas loading reached the record's ceiling. Measured on this
    // exact profile: 3.30 m before the bus died, 5.07 m in the blind row.
    if (rowBlind.ceilingCm <= rowHonest.ceilingCm)
        FAIL("dive %d: the ceiling in the blind row is %d cm against %d cm in the last honest "
             "row — the deco picture written into the file forgot the loading the window "
             "integrated (or was measured at a depth the controller never believed)",
             id, rowBlind.ceilingCm, rowHonest.ceilingCm);
    // (3,4) Direction, not magnitude: a blind window at a held depth can only add
    // debt. If either column moved the other way, the row was totalled somewhere
    // other than the held depth.
    if (rowBlind.decoTotalMin < rowHonest.decoTotalMin)
        FAIL("dive %d: decoTotal in the blind row is %d min against %d min in the last honest "
             "row: a window that kept integrating the held depth cannot owe LESS than the "
             "honest row before it", id, rowBlind.decoTotalMin, rowHonest.decoTotalMin);
    if (rowBlind.ndlMin > rowHonest.ndlMin)
        FAIL("dive %d: NDL in the blind row is %d min against %d min in the last honest row: "
             "the file says the diver gained time while the chain was integrating a held "
             "depth", id, rowBlind.ndlMin, rowHonest.ndlMin);
    // (5) The row's pulse delta is a window difference of two absolute counters,
    // and the chain injected throughout the blind window (the setpoint stayed at
    // the held band): a row that records none has lost the delta.
    if (rowBlind.pulses == 0)
        FAIL("dive %d: the blind row records 0 injection pulses although the chain pulsed "
             "through the window (21 in the window) — the row's pulse window is not a delta "
             "of the counters", id);
    // (6) The columns are the numbers a held 32 m dive owes — measured on this
    // exact profile, 11 min of deco and 0 min of NDL in the blind row, beside 10
    // and 1 in the honest row before the fault. The direction asserts above cannot
    // see a fill that totals EVERY row at a depth the controller never believed
    // (both rows shift together); this band can, because it is an absolute claim
    // about the depth the row was written at.
    if (rowBlind.decoTotalMin < 6 || rowBlind.decoTotalMin > 25)
        FAIL("dive %d: the blind row states %d min of deco for a dive the chain was holding at "
             "32 m — the row's deco picture was totalled at some other depth",
             id, rowBlind.decoTotalMin);
    if (rowBlind.ndlMin > 10)
        FAIL("dive %d: the blind row states %d min of NDL at the depth the chain believed, on a "
             "dive that had sat at 32 m for seven minutes — the row's NDL was measured somewhere "
             "the controller was not", id, rowBlind.ndlMin);
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

// ---------------------------------------------------------------------------
// Dive 9002 — "wet connector, all three cells dead" (finding N2). The end-to-end
// witness for the ppO2 hold: the dive where the chain stops having any ppO2 it is
// willing to believe, and what the handset, the dive file, the valve, the alarms,
// the tissue model and the oxygen clock do about it.
//
// The route to that state is the physically honest one, and it takes three acts,
// because `Mode::FAILED` with cells seen is not somewhere a dead cell walks you:
//   1. water in the cell harness. Two of three galvanic channels are frozen at
//      plausible-but-wrong values (each biased differently — a flooded connector
//      does not bias channels equally), the third is still honest. What the chain
//      does here is NOT what a naive model expects: the two channels do not drag
//      the median down with them, because a 0.5 bar step in one second is past
//      voting's dropout slope, so those channels are dropped from the vote
//      immediately and the dive goes on with one honest cell and CELL_DRIFT up.
//      (The median-follows-the-liars shape is real too, and it is a CREEP, not a
//      step: the soak's F_TWIN_DRIFT dives own that one.)
//   2. CellProof's re-arm is the machine's next move, and the dive needs its
//      verdict: a REJECTED cell arguing with the vote by ~0.5 bar for 20 s is the
//      "your cell picture is stale" evidence, the re-run judges every fitted cell
//      on whether it TRACKED the demand change, and the two frozen channels never
//      moved — so they come back marked unproven-at-high and proof-banned. That
//      ban is what makes act 3 a failed vote instead of an argument.
//   3. then the connector takes the last channel's ADC channel with it. The cell
//      is fine — the machine simply stops being told what it says — so the only
//      fitted cells left are the two it has evidence against, nothing may vote,
//      and the vote FAILS with cells seen: BIT_SENSOR_FAIL. That is the instant
//      finding N2 is about. Until 2026-11-26 the handset's ppO2 column and every
//      dive-file row of the window said 0.00 — a fabricated catastrophic hypoxia,
//      argued with the machine's own "I cannot see your loop" alarm. Now the
//      number is HELD and the marker bytes say so.
//
// Act 3 needs `Rig::setCellsFitted` and it is worth stating what that means for
// shipping firmware: the mask there is a boot constant (main.cpp CELLS_PRESENT_MASK),
// so a real diver reaches this state only if the fitted set changes mid-dive (a
// SET, or a future hot-swap path). Reported as an audit finding, not hidden: the
// dive grades the chain's behaviour in the state, and the reachability of the
// state in the shipped configuration is a separate question with a separate answer.
//
// The twin, as in dive 9001: same options, same diver, same true depth every
// tick, honest harness. The two dives are identical until act 1, so a difference
// at the end of the blind window is the fault and nothing else — which is what
// lets the deco asserts below assert a DIRECTION (the blind rig must be the
// conservative one) instead of a hard-coded tissue number.
// ---------------------------------------------------------------------------
void runWetConnectorDive() {
    const int id = 9002;
    constexpr float kBottomM = 32.0f;    // SP3 band: 1.00 bar, deep enough to matter
    constexpr float kWetAtS = 430.0f;    // steady on the bottom: deadlines mean something
    constexpr float kBlindAtS = 570.0f;  // act 3, after the proof marked the frozen pair
    // 65 s with no believable ppO2: long enough to fill two whole 30 s dive-file
    // rows, short enough that the loop is still survivable when it ends. At 32 m the
    // diver burns 1.2 l/min x 4.2 bar out of 8 l, so a blind minute takes ~0.6 bar of
    // the loop — which is the honest consequence of this fault and worth stating: the
    // machine cannot hold a loop it cannot see, its answer is the CRITICAL alarm, and
    // the diver's answer is bailout. Grading that the loop stayed perfect would be
    // grading a fiction.
    constexpr float kWindowS = 65.0f;
    // Frozen channels, biased apart (0.45 is the agreed 2-vs-1 lie depth; 0.52 keeps
    // the two frozen readings apart from each other, which is what a connector does).
    // Both are well inside the plausibility band: these are not readings the chain can
    // dismiss as noise, they are a loop that is no longer being measured.
    constexpr float kBiasCh0 = 0.45f, kBiasCh1 = 0.52f;
    // The chain notices the collapse quickly (the step is past the dropout slope):
    // a few cycles to drop the channels, then the 20 s suspect hold and the 60 s
    // proof run bring the ban. Measured on this dive: news ~1 s, ban ~22 s.
    constexpr float kNewsDeadlineS = 10.0f;
    constexpr float kBanDeadlineS = 200.0f;
    // BIT_SENSOR_FAIL is raised from the failed vote itself, so one tick is the
    // deadline; anything slower and the held number would sit unmarked.
    constexpr float kSensorFailDeadlineS = 1.0f;
    constexpr float kRecoverDeadlineS = 3.0f;
    // Premise of the deco asserts: by the end of the window the hold must be a
    // materially WRONG number. Injection is off for the whole window at 32 m and the
    // diver burns ~0.6 bar/min there, so the true loop ends well below the held value.
    // A held (rich) ppO2 lowers ppN2, so feeding it to the tissues would be wrong
    // in the direction that costs the diver deco time — that is the decision this
    // dive is grading (loop.h kNoVotePpo2Bar).
    constexpr float kPremiseLoopFallBar = 0.25f;
    // Over the window, ppN2 = pAmb vs ppN2 = pAmb - ~1.0 bar moves the 5-minute
    // tissue by a visible fraction of that gap. 0.08 bar is far under what the inputs produce and
    // far over what a held ppO2 would leave, so it grades the decision without
    // freezing a GF-dependent number.
    constexpr float kMinTissueGainBar = 0.08f;
    constexpr float kSpFloorBar = 0.39f, kSpCeilBar = 1.31f;

    RigOpts o = soakOpts();
    Rig rig(o);
    prediveHealthy(rig);
    if (!rig.predivePassed()) FAIL("dive %d: pre-dive failed on a rig with no fitted faults", id);
    if (!rig.requestAuto(true)) FAIL("dive %d: AUTO refused after a passed pre-dive", id);

    Rig twin(o);
    prediveHealthy(twin);
    if (!twin.predivePassed()) FAIL("dive %d: the twin's pre-dive failed", id);
    if (!twin.requestAuto(true)) FAIL("dive %d: the twin refused AUTO", id);

    rig.startDive();
    twin.startDive();

    Diver dv;
    dv.depthM[0] = kBottomM;
    dv.holdS[0] = 780.0f;  // long enough that act 1 and act 3 are both on the bottom
    dv.nWp = 1;
    dv.holdLeft = dv.holdS[0];

    const float clearAtS = kBlindAtS + kWindowS;
    const float dtS = Rig::kTickMs / 1000.0f;
    const float ackDelayS = 8.0f;

    // ---- state the grading needs ------------------------------------------
    bool wetDone = false, banSeen = false, blindOpen = false, blindClosed = false;
    bool voteBack = false, pulsesResumed = false, cpAttemptsChecked = false;
    float wetAtS = 0.0f, banAtS = -1.0f, newsAtS = -1.0f, voteBackS = -1.0f;
    float heldPpo2 = 0.0f;                 // the last believable vote, shown all window
    float pTrueAtOpen = 0.0f, pTrueMinInWindow = 9.0f;
    float heldSeenMin = 9.0f, heldSeenMax = -1.0f;  // the published number, cycle by cycle
    int failAtCycle = -1, windowCycle = 0;
    int lowDuring = 0, highDuring = 0, pulsesInWindow = 0, twinPulsesInWindow = 0;
    uint8_t cpAttemptsAtOpen = 0;
    double fastOpen = 0.0, fastClose = 0.0, twinFastOpen = 0.0, twinFastClose = 0.0;
    double cnsOpen = 0.0, cnsClose = 0.0, twinCnsOpen = 0.0, twinCnsClose = 0.0;
    double ceilClose = 0.0, twinCeilClose = 0.0;
    uint32_t pulsesAtOpen = 0, twinPulsesAtOpen = 0;
    // Rows, built by the firmware's own fill exactly as main.cpp calls it.
    logfmt::Snapshot row{}, rowHonest{}, rowBlind{};
    uint32_t rowPulseBase = 0;
    float rowClockS = 0.0f;
    int rowsBlind = 0, rowsBlindMarker = 0, blindCyclesInRow = 0;
    int ackSilencedCycles = 0;
    bool logFailAfterRecovery = false;
    uint16_t alarmUnion = 0;
    float alarmSinceS = -1.0f;
    bool finished = false;
    float tS = 0.0f;

    for (; tS < 2400.0f && !finished; tS += dtS) {
        // ---- act 1: water in the harness ---------------------------------
        if (!wetDone && tS >= kWetAtS) {
            const float voted = rig.vote().votedPpo2;
            if (!rig.loop().haveVote() || voted < 0.6f)
                FAIL("dive %d: the vote was %.2f bar (haveVote=%d) when the connector was "
                     "flooded — the scenario is starting from the wrong state",
                     id, voted, (int)rig.loop().haveVote());
            if (std::fabs(rig.deco().tissueBar(0) - twin.deco().tissueBar(0)) > 0.02)
                FAIL("dive %d: twin (%.2f bar) and faulted rig (%.2f bar) already differed when "
                     "the connector was flooded — the comparison below is not a control",
                     id, twin.deco().tissueBar(0), rig.deco().tissueBar(0));
            World& w = rig.world();
            w.cells[0].fault = CellFault::STUCK_AT;
            w.cells[0].stuckValue = voted - kBiasCh0;
            w.cells[1].fault = CellFault::STUCK_AT;
            w.cells[1].stuckValue = voted - kBiasCh1;
            wetDone = true;
            wetAtS = tS;
        }

        // Did the machine notice the two channels going deaf to the loop? Either
        // news counts, because both are things the diver is told about.
        if (wetDone && newsAtS < 0.0f &&
            ((rig.alarmLogBits() & (uint16_t)(1u << alarms::BIT_CELL_DRIFT)) ||
             !(rig.vote().activeMask & 0b011)))
            newsAtS = tS;

        // ---- act 2 is the machine's own: the proof re-arms and bans them --
        if (!banSeen && rig.cpState() == cellproof::State::PASS &&
            rig.cpUnprovenMask() == 0b011 && rig.proofExclusion() == 0b011 &&
            rig.loop().haveVote()) {
            banSeen = true;
            banAtS = tS;
        }

        // ---- act 3: the last channel's channel goes deaf ------------------
        if (!blindOpen && tS >= kBlindAtS) {
            if (!banSeen)
                FAIL("dive %d: %.0f s after the connector was flooded, the proof had not "
                     "banned the two frozen channels (cp=%d unproven=0o%o ban=0o%o vote=%.2f) — "
                     "without a proof ban on the fitted set there is no failed vote here, only "
                     "a vote that lies",
                     id, tS - wetAtS, (int)rig.cpState(), rig.cpUnprovenMask(),
                     rig.proofExclusion(), rig.vote().votedPpo2);
            if (banAtS - wetAtS > kBanDeadlineS)
                FAIL("dive %d: the lying pair was not marked until %.0f s after the fault "
                     "(deadline %.0f s): a diver spends that long enriching a loop the vote "
                     "says is lean", id, banAtS - wetAtS, kBanDeadlineS);
            heldPpo2 = rig.loop().shownPpo2();
            if (heldPpo2 < 0.30f)
                FAIL("dive %d: the last believable ppO2 was %.2f bar — there is no believable "
                     "number to hold, so the window below cannot grade a hold", id, heldPpo2);
            pTrueAtOpen = rig.world().ppo2True();
            fastOpen = rig.deco().tissueBar(0);
            twinFastOpen = twin.deco().tissueBar(0);
            cnsOpen = rig.cnsPct();
            twinCnsOpen = twin.cnsPct();
            pulsesAtOpen = rig.pulseCount();
            twinPulsesAtOpen = twin.pulseCount();
            cpAttemptsAtOpen = rig.loop().cellproof().attempts;
            blindOpen = true;
            // The channel that goes deaf is the one the whole dive has been voting
            // on. The cell is not broken: the machine stops being told what it says.
            rig.setCellsFitted(0b011);
        }
        if (blindOpen && !blindClosed && tS >= clearAtS) {
            rig.setCellsFitted(0b111);
            fastClose = rig.deco().tissueBar(0);
            twinFastClose = twin.deco().tissueBar(0);
            cnsClose = rig.cnsPct();
            twinCnsClose = twin.cnsPct();
            ceilClose = rig.deco().ceilingM();
            twinCeilClose = twin.deco().ceilingM();
            pulsesInWindow = (int)(rig.pulseCount() - pulsesAtOpen);
            twinPulsesInWindow = (int)(twin.pulseCount() - twinPulsesAtOpen);
            blindClosed = true;
        }

        // ---- the rig and its twin, at the same true depth -----------------
        rig.world().setDepthM(dv.depth);
        twin.world().setDepthM(dv.depth);
        rig.tick();
        twin.tick();

        const uint16_t live = rig.alarmBits();
        const uint16_t logv = rig.alarmLogBits();
        alarmUnion |= live;
        const float sp = rig.spTarget();
        if (sp < kSpFloorBar || sp > kSpCeilBar)
            FAIL("dive %d: setpoint escaped to %.2f bar at t=%.0fs depth=%.1fm", id, sp, tS,
                 dv.depth);
        const float pTrue = rig.world().ppo2True();
        if (pTrue < 0.12f)
            FAIL("dive %d: loop fell to %.2f bar at %.1f m (t=%.0fs alarms=0x%03x) — the "
                 "scenario drove the rig into a starvation the twin was not in", id, pTrue,
                 dv.depth, tS, alarmUnion);
        if (pTrue > 2.20f)
            FAIL("dive %d: loop reached %.2f bar (toxic) at t=%.0fs: the raw-cell high veto "
                 "let a lie about the vote drive the valve", id, pTrue, tS);
        float maxCell = 0.0f;
        for (int c = 0; c < 3; c++)
            if (rig.cellStatus((uint8_t)c).last > maxCell) maxCell = rig.cellStatus((uint8_t)c).last;
        if (maxCell >= kHighWall && rig.injOut().pulseStarted)
            FAIL("dive %d: pulse started while a cell read %.2f bar (wall %.2f) at t=%.0fs",
                 id, maxCell, kHighWall, tS);

        // ---- inside the blind window, cycle by cycle ----------------------
        if (blindOpen && !blindClosed) {
            windowCycle++;
            if (pTrue < pTrueMinInWindow) pTrueMinInWindow = pTrue;
            if (rig.state().shownPpo2 < heldSeenMin) heldSeenMin = rig.state().shownPpo2;
            if (rig.state().shownPpo2 > heldSeenMax) heldSeenMax = rig.state().shownPpo2;
            if (rig.state().haveVote)
                FAIL("dive %d: the vote came back at t=%.0fs with every fitted cell barred — "
                     "either the ban leaked or the failed set is not what this dive thinks",
                     id, tS);
            // What the chain publishes and what the chain holds must be the same
            // number: main.cpp feeds the handset, the HUD and the dive file from
            // State, so a hold that lives only in the accessor would be invisible
            // on the wire and this dive would grade the wrong thing.
            if (std::fabs(rig.state().shownPpo2 - rig.loop().shownPpo2()) > 1e-6f)
                FAIL("dive %d: the published State shows %.3f ppO2 while the chain holds %.3f — "
                     "the number a diver downloads is not the number the chain decided on",
                     id, rig.state().shownPpo2, rig.loop().shownPpo2());
            // The published number: held, not 0.00, and not moving.
            if (std::fabs(rig.state().shownPpo2 - heldPpo2) > 1e-6f)
                FAIL("dive %d: the published ppO2 moved to %.3f during a failed vote (held "
                     "%.3f): the handset number is being made from something other than the "
                     "last believable vote", id, rig.state().shownPpo2, heldPpo2);
            if (rig.vote().votedPpo2 != 0.0f)
                FAIL("dive %d: a failed vote reported %.2f bar to the consumers gated on "
                     "haveVote — the hold belongs to the display, not to the measurement",
                     id, rig.vote().votedPpo2);
            // The marker has to be there from the first cycle, or the held number
            // is an unmarked claim for as long as it takes to arrive.
            if (live & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) {
                if (failAtCycle < 0) failAtCycle = windowCycle;
            }
            if (windowCycle * dtS >= kSensorFailDeadlineS && failAtCycle < 0)
                FAIL("dive %d: %.0f s into a failed vote with cells seen, BIT_SENSOR_FAIL had "
                     "not been raised (alarms=0x%03x): the held ppO2 arrived with nothing "
                     "saying it is held", id, windowCycle * dtS, live);
            if (!(logv & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
                FAIL("dive %d: the LOG view lost BIT_SENSOR_FAIL at t=%.0fs while the condition "
                     "was still standing — the dive file's marker is not a latch, it is the fact",
                     id, tS);
            // Level alarms are statements about a measurement, and there is none.
            if (live & (uint16_t)(1u << alarms::BIT_LOW_PPO2)) lowDuring++;
            if (live & (uint16_t)(1u << alarms::BIT_HIGH_PPO2)) highDuring++;
            // Injection: stopped. The held value sits far below the setpoint, so a
            // hold that reached the valve would pulse the whole window.
            if (rig.pulseCount() != pulsesAtOpen)
                FAIL("dive %d: the valve pulsed at t=%.0fs with no believable ppO2 (held "
                     "%.2f bar vs setpoint %.2f): the shown number drove injection",
                     id, tS, rig.loop().shownPpo2(), sp);
            if (rig.loop().cellproof().attempts != cpAttemptsAtOpen && !cpAttemptsChecked) {
                cpAttemptsChecked = true;
                FAIL("dive %d: CellProof re-armed (%d -> %d attempts) inside the blind window: "
                     "this dive would be grading a proof run instead of the no-vote path",
                     id, (int)cpAttemptsAtOpen, (int)rig.loop().cellproof().attempts);
            }
        }
        if (blindClosed && !voteBack && rig.loop().haveVote()) {
            voteBack = true;
            voteBackS = tS;
        }
        if (voteBack && rig.pulseCount() > pulsesAtOpen && !pulsesResumed) pulsesResumed = true;
        if (blindClosed && tS > clearAtS + 5.0f &&
            (logv & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
            logFailAfterRecovery = true;

        // ---- the simulated diver -----------------------------------------
        const bool bits = live != 0;
        if (bits && alarmSinceS < 0.0f) alarmSinceS = tS;
        if (!bits) alarmSinceS = -1.0f;
        if (alarmSinceS >= 0.0f && tS - alarmSinceS >= ackDelayS) {
            rig.ackAlarms();
            alarmSinceS = -1.0f;
        }
        if (blindOpen && !blindClosed &&
            !(live & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) &&
            (logv & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
            ackSilencedCycles++;

        // ---- the 30 s dive-file row, by the firmware's own fill -----------
        if (blindOpen && !blindClosed) blindCyclesInRow++;
        if (tS >= rowClockS) {
            // Same mapping main.cpp makes, from the same source it reads: the
            // published State. The row is then the record the controller would
            // have written, not a paraphrase of the chain's internals.
            const ctrl::State& st = rig.state();
            logfmt::RowInputs ri{};
            ri.nowMs = static_cast<uint32_t>(tS * 1000.0f);
            ri.diveStartMs = 0;
            ri.smoothedDepthM = st.depthSmoothedM;
            ri.haveVote = st.haveVote;
            ri.shownPpo2 = st.shownPpo2;
            for (int c = 0; c < 3; c++)
                ri.cellBar[c] = rig.cellStatus(static_cast<uint8_t>(c)).last;
            ri.setpointBar = st.setpointBar;
            ri.diveActive = true;
            ri.autoMode = st.autoMode;
            ri.alarmLogBits = st.alarms.logBits;
            const long long cnsX = numfmt::scaled(rig.cnsPct(), 0);
            ri.cnsPeakPct = static_cast<uint8_t>(cnsX > 254 ? 254 : cnsX);
            ri.vbat = VBAT_CELLS * 3.90f;
            ri.pulsesNow = rig.pulseCount();
            ri.pulsesPrev = rowPulseBase;
            // The published picture, computed exactly the way ControlTask
            // computes it (deco/view.h): the soak mirrors the PUBLICATION,
            // not the engine walk — since N8 the engine walk does not exist.
            ri.hasDecoPicture = true;
            deco::viewFull(ri.deco, rig.loop().deco(),
                           rig.loop().deco().depthToBar(ri.smoothedDepthM),
                           ri.haveVote, ri.shownPpo2);
            logfmt::fillSnapshot(row, ri);
            rowPulseBase = ri.pulsesNow;
            if (!blindOpen) rowHonest = row;
            if (blindCyclesInRow >= 150 && blindOpen && !blindClosed) {
                rowsBlind++;
                rowBlind = row;
                if (row.alarmBits & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) rowsBlindMarker++;
            }
            rowClockS += 30.0f;
            blindCyclesInRow = 0;
        }

        // ---- the diver moves (under the faulted machine's own ceiling) ----
        if (dv.step(dtS, rig.deco(), tS)) finished = true;
    }
    rig.endDive();
    twin.endDive();

    // ---- grading ----------------------------------------------------------
    if (!wetDone) FAIL("dive %d: the connector was never flooded", id);
    if (!blindClosed) FAIL("dive %d: the blind window never opened/closed (blind=%d closed=%d)",
                           id, (int)blindOpen, (int)blindClosed);
    if (newsAtS < 0.0f || newsAtS - wetAtS > kNewsDeadlineS)
        FAIL("dive %d: %d s after two channels froze at plausible-but-wrong values the chain "
             "had neither dropped them from the vote nor raised CELL_DRIFT (news at %.0f s, "
             "activeMask 0o%o) — act 3 would be built on a fault nobody saw",
             id, (int)(tS - wetAtS), newsAtS, (unsigned)rig.vote().activeMask);
    if (failAtCycle < 0) FAIL("dive %d: BIT_SENSOR_FAIL was never raised in the window", id);
    if (failAtCycle * dtS > kSensorFailDeadlineS)
        FAIL("dive %d: BIT_SENSOR_FAIL took %d cycles to follow a failed vote: the held ppO2 "
             "was on the handset unmarked for that long", id, failAtCycle);
    if (heldSeenMax - heldSeenMin > 1e-6f)
        FAIL("dive %d: the published ppO2 drifted across the window (%.3f -> %.3f) — a held "
             "number that moves is a measurement being guessed", id, heldSeenMin, heldSeenMax);
    if (std::fabs(heldSeenMin - heldPpo2) > 1e-6f)
        FAIL("dive %d: the window showed %.3f bar, not the last believable vote (%.3f)",
             id, heldSeenMin, heldPpo2);
    if (lowDuring || highDuring)
        FAIL("dive %d: a level alarm (LOW %d cycles, HIGH %d cycles) stood on a ppO2 that was "
             "no longer a measurement — the failure alarm is what the diver is owed, not a "
             "verdict about a number the chain stopped measuring", id, lowDuring, highDuring);
    if (pulsesInWindow != 0)
        FAIL("dive %d: %d injection pulses with no believable ppO2: the held %.2f bar was "
             "driving the valve (the twin pulsed %d times at the same depth)", id,
             pulsesInWindow, heldPpo2, twinPulsesInWindow);
    if (twinPulsesInWindow <= 0)
        FAIL("dive %d: the twin pulsed %d times across the window, so the assert above cannot "
             "tell a stopped valve from a stopped dive", id, twinPulsesInWindow);

    // The premise the deco direction depends on: the hold has to be a materially
    // wrong number by the end of the window. The loop fell while the valve was shut;
    // the machine kept showing the number it had before the fall.
    if (pTrueAtOpen - pTrueMinInWindow < kPremiseLoopFallBar)
        FAIL("dive %d: the loop only fell %.2f bar during %0.0f s of no injection (%.2f -> %.2f), "
             "so a held ppO2 and kNoVotePpo2Bar would barely differ and the deco asserts below "
             "prove nothing", id, pTrueAtOpen - pTrueMinInWindow, kWindowS, pTrueAtOpen,
             pTrueMinInWindow);

    // Tissues: decision (a) of finding N2 — no vote, tissues loaded at
    // kNoVotePpo2Bar (ppN2 = pAmb, the most inert gas this depth allows). The
    // direction is graded against the twin over the same span at the same true
    // depth; the held ppO2 (0.45..1.3 bar rich) would have UNDER-recorded.
    if (!(rig.deco().tissueBar(0) > twin.deco().tissueBar(0)))
        FAIL("dive %d: the blind rig's fast tissue (%.3f bar) is not above the twin's (%.3f) — "
             "the window's gas loading is not reaching the model", id, rig.deco().tissueBar(0),
             twin.deco().tissueBar(0));
    const double gainBlind = fastClose - fastOpen, gainTwin = twinFastClose - twinFastOpen;
    if (!(gainBlind >= gainTwin + kMinTissueGainBar))
        FAIL("dive %d: across the %.0f s no-vote window the fast tissue gained %.3f bar against "
             "the twin's %.3f bar at the same depth: the tissues are being fed the held ppO2 "
             "instead of kNoVotePpo2Bar, which under-records decompression debt",
             id, kWindowS, gainBlind, gainTwin);
    if (!(ceilClose >= twinCeilClose))
        FAIL("dive %d: the ceiling after the window is %.1f m against the twin's %.1f m — the "
             "conservative no-vote loading has to leave the diver with at least as much deco",
             id, ceilClose, twinCeilClose);

    // CNS: frozen with nothing to integrate. This is the one window in this dive
    // that UNDER-records, so it is stated in loop.cpp step 8 and pinned here: if the
    // rule changes, this reddens and the residual has to be re-argued, not lost.
    if (!(std::fabs(cnsClose - cnsOpen) < 1e-9))
        FAIL("dive %d: the oxygen clock advanced %.4f %% during a failed vote — the held ppO2 "
             "reached stepCns, or the frozen-clock residual changed silently",
             id, cnsClose - cnsOpen);
    if (!(twinCnsClose > twinCnsOpen + 0.01))
        FAIL("dive %d: the twin's CNS moved %.4f %% over the same span, so the frozen clock "
             "above may just be a stopped dive", id, twinCnsClose - twinCnsOpen);

    // The dive file: the rows the diver would download out of that window.
    if (rowsBlind == 0)
        FAIL("dive %d: no dive-file row was wholly inside the %.0f s no-vote window, so the "
             "record of it is ungraded", id, kWindowS);
    if (rowBlind.votedX1000 <= 0)
        FAIL("dive %d: the dive file recorded %.3f ppO2 in the no-vote window — the handset was "
             "showing %.3f, and 0.000 is the fabricated hypoxia this finding closed",
             id, (float)rowBlind.votedX1000 / 1000.0f, heldPpo2);
    const int32_t wantX = static_cast<int32_t>(numfmt::scaled(heldPpo2, 3));
    if (std::abs(rowBlind.votedX1000 - wantX) > 1)
        FAIL("dive %d: the row's ppO2 column is %.3f, not the held value %.3f the chain was "
             "publishing at the same instant", id, (float)rowBlind.votedX1000 / 1000.0f, heldPpo2);
    if (!(rowBlind.alarmBits & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
        FAIL("dive %d: the no-vote row carries a ppO2 that is not a measurement with nothing "
             "saying so (alarm word 0x%03x): BIT_SENSOR_FAIL is the marker", id,
             rowBlind.alarmBits);
    if (rowBlind.ndlMin != deco::kNDL_UNKNOWN)
        FAIL("dive %d: the no-vote row reports NDL %d instead of the unknown sentinel (-1, "
             "the one pair for file and wire since review N5): the held ppO2 was allowed to "
             "reduce the diver's no-deco time", id, rowBlind.ndlMin);
    if (rowHonest.ndlMin == deco::kNDL_UNKNOWN)
        FAIL("dive %d: the last honest row already carried the unknown sentinel, so the "
             "sentinel assert above distinguishes nothing", id);
    if (rowsBlindMarker != rowsBlind)
        FAIL("dive %d: only %d of %d no-vote rows carried BIT_SENSOR_FAIL: the file marks some "
             "of the held numbers and not others", id, rowsBlindMarker, rowsBlind);
    if (ackSilencedCycles == 0)
        FAIL("dive %d: across %d no-vote rows the ACK never took BIT_SENSOR_FAIL off the live "
             "word while the log view kept it — nothing here separates the two views", id,
             rowsBlind);
    if (logFailAfterRecovery)
        FAIL("dive %d: 5 s after the channel came back, the dive-file view still claimed "
             "BIT_SENSOR_FAIL — the marker is a fact about now, not a latch", id);

    // Recovery: the channel returns, the vote returns with it, the shown number
    // steps to the fresh measurement, and control takes the valve back.
    if (!voteBack)
        FAIL("dive %d: the vote never came back when the channel did", id);
    if (voteBackS - clearAtS > kRecoverDeadlineS)
        FAIL("dive %d: the vote took %.1f s to return after the channel did (deadline %.1f s)",
             id, voteBackS - clearAtS, kRecoverDeadlineS);
    if (rig.state().shownPpo2 <= 0.001f)
        FAIL("dive %d: the vote is back and the shown ppO2 is still %.3f — the hold outlived "
             "the reason for it", id, rig.state().shownPpo2);
    if (std::fabs(rig.state().shownPpo2 - rig.vote().votedPpo2) > 1e-6f)
        FAIL("dive %d: with a vote running the shown ppO2 (%.3f) is not the vote (%.3f)",
             id, rig.state().shownPpo2, rig.vote().votedPpo2);
    if (!pulsesResumed)
        FAIL("dive %d: injection never resumed after the vote did (loop at %.2f bar, setpoint "
             "%.2f)", id, rig.world().ppo2True(), rig.spTarget());
    if (!(rig.proofExclusion() & 0b011))
        FAIL("dive %d: the proof ban on the two frozen channels was lifted by the channel "
             "coming back — a cell the machine has evidence against is back in the vote", id);

    std::printf("dive %d  wet-connector  %5.0f m  flooded %3.0f s -> noticed %3.0f s -> banned "
                "%3.0f s -> blind "
                "%3.0f s  held ppO2 %.3f (loop fell to %.2f)  pulses %d vs twin %d  "
                "CNS %+.2f%% vs twin %+.2f%%  fast tissue %+.2f vs twin %+.2f  ceil %.1f vs "
                "twin %.1f m  rows %d (all marked, NDL %d)  no-vote cycles %d  alarms 0x%03x\n",
                id, kBottomM, wetAtS, newsAtS, banAtS, kWindowS, heldPpo2, pTrueMinInWindow,
                pulsesInWindow, twinPulsesInWindow, cnsClose - cnsOpen,
                twinCnsClose - twinCnsOpen, gainBlind, gainTwin, ceilClose, twinCeilClose,
                rowsBlind, (int)rowBlind.ndlMin, (int)windowCycle, alarmUnion);
    std::printf("dive %d  ppO2-view  %d/%d no-vote dive-file rows hold %.3f and carry "
                "BIT_SENSOR_FAIL across the ACK  (live word silent for %d cycles, level alarms "
                "quiet, valve frozen for %.0f s, log view clear after recovery)\n",
                id, rowsBlindMarker, rowsBlind, heldPpo2, ackSilencedCycles, kWindowS);
}

// ---------------------------------------------------------------------------
// Dive 9003 — "three channels freeze at plausible values" (residual #2 of the
// held-ppO2 batch, the maintainer's headline hole: dead cells reading a
// PLAUSIBLE number keep the vote alive, and that is a bigger safety hole than
// the display bug it was born from).
//
// 9002 froze channels with a STEP: the collapse slope was itself the news, the
// dropout gate caught it in about a second, and the dive graded the proof
// re-arm and the no-vote machinery. This dive is the shape NONE of those gates
// can see: the readings stop moving where they stand. Plausible before,
// plausible after, no step, no drift, no collapse — the only evidence that the
// channels died is that the ADC dither the bench proves every live channel has
// (4..7 counts p-p at rest, 2026-11-26) went silent.
//
// Acts:
//   1. A long, quiet, HEALTHY hold at 30 m — deliberately two full detector
//      windows long, with an honest twin on the same profile: the frozen
//      detector must never condemn a live chain that simply has nothing to
//      report. A false accusation here reddens the dive outright.
//   2. Mid-dive a wet connector freezes ALL THREE channels at the values they
//      were just reporting (sim: CellFault::STUCK_AT at the live value +
//      CellModel::noDither — the physical latch-up, not the 9002 step). No
//      gate but the frozen-reading detector sees anything. Within one
//      stuckWindow (20 s) every channel must be excluded on evidence —
//      barReason STUCK — the vote must go FAILED with cellsSeen true, and
//      BIT_SENSOR_FAIL must fire within a cycle of that: DETECTION, not the
//      bare survival 9002 bought with a proof ban.
//   3. The held-ppO2 machinery (dive 9002's finding) must hold the last
//      believable number, mark it, keep the valve shut, keep the tissues
//      loading conservatively, and the loop must not go toxic while the frozen
//      vote was still outvoting reality for those 20 s.
//   4. The connector dries: dither returns, each cell REFUTES its own frozen
//      bar by moving, and the vote comes back. The detector's honesty runs
//      both ways.
//
// CellProof is OFF on this rig on purpose: this dive grades the voting layer's
// own evidence chain. 9002 remains the dive that grades the proof re-arm.
// ---------------------------------------------------------------------------
void runFrozenChannelDive() {
    const int id = 9003;

    constexpr float kBottomM = 30.0f;
    constexpr float kFreezeAtS = 300.0f;   // long after anything pre-dive started
    // The detector needs one full stuckWindow of stillness (200 fresh samples,
    // one tick each here); allow a small alignment margin.
    // The observed detection lands at 20.1 s (one stuckWindow of fresh ticks +
    // the band-alignment sample); 21 s is that plus one cycle of margin — a
    // deadline this tight is the point: every extra second is a second three
    // frozen channels were still driving the valve.
    constexpr float kDetectDeadlineS = 21.0f;
    // The no-vote window. Long enough for two 30 s dive-file rows to fall
    // wholly inside it and for the loop to fall materially (the premise the
    // deco asserts grade); short enough that the valve-shut loop — the point
    // of the window — never reaches the starvation the fault-free twin never
    // approaches.
    constexpr float kWindowS = 30.0f;
    constexpr float kSensorFailDeadlineS = 1.0f;
    constexpr float kRecoverDeadlineS = 3.0f;
    // The 20 s between freeze and detection are NOT harmless: the fossil vote
    // still drives the valve. The loop must stay inside the band the level
    // alarms and the walls police.
    // During those 20 s the fossil vote is still believed, so the loop must be
    // kept inside the band a healthy machine keeps it in: it may not be enriched
    // toward toxicity NOR already collapsing (a collapse would mean the frozen
    // values were fiction from the instant of the fault, and then the dropout
    // and level gates — not the frozen detector — would be doing the work).
    constexpr float kHazardHighBar = 1.60f, kHazardLowBar = 0.30f;
    // After detection the valve is SHUT by design, so the loop DOES fall — that
    // is 9002's premise, and the window asserts grade it. What must never
    // happen anywhere is the true starvation the twin never enters.
    // 30 s of SAC in an 8 L loop at 30 m is 0.45 bar of oxygen, so this dive
    // legitimately rides down toward the soak's usual 0.12 bar floor — the
    // valve is shut BY DESIGN for the window, that is the finding. The floor
    // asserted here sits under that dive bottom but far above the ~0.05 bar a
    // diluent flush reaches and far above death: what this dive must never do
    // is drown the rig while claiming to grade a detector.
    constexpr float kStarvationFloorBar = 0.12f;
    // Premise for the deco-direction asserts: injection is shut for the whole
    // window at 30 m, the diver burns ~0.4 bar/min there, and the loop must end
    // materially under the held value.
    constexpr float kPremiseLoopFallBar = 0.25f;
    // Over this 30 s window the no-vote rig's fast tissue gains ~0.15 bar vs
    // the twin's ~0.10 (the kNoVotePpo2Bar over-recording, ppN2 = pAmb vs
    // pAmb - ppO2). The window is short — bounded by how long a real loop may
    // go without injection at this SAC — so the differential is smaller than
    // 9002's 65 s window shows; 0.04 grades the DIRECTION with ~2x margin
    // without freezing a window-length-dependent number.
    constexpr float kMinTissueGainBar = 0.04f;
    constexpr float kSpFloorBar = 0.39f, kSpCeilBar = 1.31f;

    RigOpts o = soakOpts();
    o.cellproof.enabled = false;  // grade the VOTING layer; 9002 owns the proof path
    Rig rig(o);
    prediveHealthy(rig);
    if (!rig.predivePassed()) FAIL("dive %d: pre-dive failed on a rig with no fitted faults", id);
    if (!rig.requestAuto(true)) FAIL("dive %d: AUTO refused after a passed pre-dive", id);

    Rig twin(o);
    prediveHealthy(twin);
    if (!twin.predivePassed()) FAIL("dive %d: the twin's pre-dive failed", id);
    if (!twin.requestAuto(true)) FAIL("dive %d: the twin refused AUTO", id);

    rig.startDive();
    twin.startDive();

    Diver dv;
    dv.depthM[0] = kBottomM;
    dv.holdS[0] = 700.0f;  // the whole dive on one bottom: acts 1-4 all fit
    dv.nWp = 1;
    dv.holdLeft = dv.holdS[0];

    const float clearAtS = kFreezeAtS + kDetectDeadlineS + kWindowS;  // thaw time
    const float dtS = Rig::kTickMs / 1000.0f;
    const float ackDelayS = 8.0f;

    // ---- state the grading needs ------------------------------------------
    bool frozen = false, detected = false, blindOpen = false, blindClosed = false;
    bool voteBack = false, pulsesResumed = false;
    float freezeAtS = 0.0f, detectAtS = -1.0f, voteBackS = -1.0f;
    float heldPpo2 = 0.0f;
    float pTrueAtOpen = 0.0f, pTrueMinInWindow = 9.0f, pTrueMaxInHazard = 0.0f;
    float heldSeenMin = 9.0f, heldSeenMax = -1.0f;
    int failAtCycle = -1, windowCycle = 0, hazardCycles = 0;
    int lowDuring = 0, highDuring = 0, pulsesInWindow = 0, twinPulsesInWindow = 0;
    double fastOpen = 0.0, fastClose = 0.0, twinFastOpen = 0.0, twinFastClose = 0.0;
    double cnsOpen = 0.0, cnsClose = 0.0, twinCnsOpen = 0.0, twinCnsClose = 0.0;
    double ceilClose = 0.0, twinCeilClose = 0.0;
    uint32_t pulsesAtOpen = 0, twinPulsesAtOpen = 0;
    logfmt::Snapshot row{}, rowHonest{}, rowBlind{};
    uint32_t rowPulseBase = 0;
    float rowClockS = 0.0f;
    int rowsBlind = 0, rowsBlindMarker = 0, blindCyclesInRow = 0;
    int ackSilencedCycles = 0;
    uint16_t alarmUnion = 0;
    float alarmSinceS = -1.0f;
    bool finished = false;
    float tS = 0.0f;

    for (; tS < 1100.0f && !finished; tS += dtS) {
        // ---- act 1 guard, every cycle from the first: the twin (and the rig
        // before the fault) is a LIVE chain in a quiet hold. A stuck
        // accusation on either before the fault is a false positive, full
        // stop — this is the batch's honesty rule, not a race to win.
        if (twin.vote().activeMask != 0b111)
            FAIL("dive %d: the fault-free twin lost a cell at t=%.0fs (act=%o, bar %d/%d/%d) — "
                 "the frozen detector condemned a live, quiescent chain",
                 id, tS, (unsigned)twin.vote().activeMask,
                 twin.cellBarReason(0), twin.cellBarReason(1), twin.cellBarReason(2));
        if (twin.alarmBits() & (uint16_t)((1u << alarms::BIT_SENSOR_FAIL) | (1u << alarms::BIT_CELL_DRIFT)))
            FAIL("dive %d: the fault-free twin raised SENSOR_FAIL/CELL_DRIFT at t=%.0fs "
                 "(alarms 0x%03x) — quiet is not evidence of a frozen channel",
                 id, tS, twin.alarmBits());
        if (!frozen && rig.vote().activeMask != 0b111)
            FAIL("dive %d: the rig lost a cell at t=%.0fs before any fault was injected "
                 "(act=%o) — the detector false-positived on a healthy hold",
                 id, tS, (unsigned)rig.vote().activeMask);

        // ---- act 2: the connector latches — all three channels go still ---
        if (!frozen && tS >= kFreezeAtS) {
            const float voted = rig.vote().votedPpo2;
            if (!rig.loop().haveVote() || voted < 0.5f)
                FAIL("dive %d: the vote was %.2f bar (haveVote=%d) when the channels froze — "
                     "the scenario must start from a healthy, plausible loop",
                     id, voted, (int)rig.loop().haveVote());
            if (std::fabs(rig.deco().tissueBar(0) - twin.deco().tissueBar(0)) > 0.02)
                FAIL("dive %d: twin (%.2f bar) and faulted rig (%.2f bar) already differed at "
                     "the freeze — the comparison below is not a control",
                     id, twin.deco().tissueBar(0), rig.deco().tissueBar(0));
            World& w = rig.world();
            for (int c = 0; c < 3; c++) {
                // Freeze at the value the channel is reporting RIGHT NOW: no
                // step, no level change — only the wobble stops. tauS small so
                // the internal state pins to the captured value within a tick,
                // noDither because a latched input does not wobble.
                w.cells[c].fault = CellFault::STUCK_AT;
                w.cells[c].stuckValue = w.cells[c].state;
                w.cells[c].tauS = 0.3f;
                w.cells[c].noDither = true;
            }
            frozen = true;
            freezeAtS = tS;
        }

        // The freeze must be SILENT: if the dropout/level/drift gates saw it,
        // this dive is grading 9002's path again, not the frozen detector.
        if (frozen && !detected) {
            hazardCycles++;
            if (rig.vote().dropoutEvent)
                FAIL("dive %d: the frozen channels produced a dropout EVENT at t=%.0fs — the "
                     "freeze was a step, and this dive is no longer grading the frozen-reading "
                     "detector", id, tS);
            // A cell may leave the vote BEFORE the vote fails only to some other
            // gate. The frozen detector's own verdict (the whole vote gone at
            // once, all bars STUCK) is not a violation — the detection block
            // below grades it in full.
            if (!(rig.vote().activeMask & 0b111) && rig.vote().mode != voting::Mode::FAILED)
                FAIL("dive %d: a channel left the vote at t=%.0fs before the stuck window "
                     "completed (act=%o, mode=%d) — which gate took it? this dive injects a "
                     "silent freeze, not a collapse", id, tS, (unsigned)rig.vote().activeMask,
                     (int)rig.vote().mode);
        }

        // ---- act 2 verdict: detection ON EVIDENCE, within one window ------
        if (frozen && !detected && rig.vote().mode == voting::Mode::FAILED) {
            detectAtS = tS;
            detected = true;
            if (tS - freezeAtS > kDetectDeadlineS)
                FAIL("dive %d: the frozen vote survived %.1f s (window is %.1f s) — the vote "
                     "on three still channels is not just stale, it is driving the valve",
                     id, tS - freezeAtS, voting::Config{}.stuckWindow * dtS);
            if (!rig.vote().cellsSeen)
                FAIL("dive %d: FAILED with cellsSeen false — the alarm layer cannot tell "
                     "blindness from boot, and the fault goes unshouted", id);
            for (int c = 0; c < 3; c++)
                if (rig.cellBarReason((uint8_t)c) != voting::Engine::BAR_STUCK)
                    FAIL("dive %d: ch%d excluded with reason %d, expected STUCK(%d): the vote "
                         "died but not on frozen-channel evidence",
                         id, c, rig.cellBarReason((uint8_t)c), (int)voting::Engine::BAR_STUCK);
            heldPpo2 = rig.loop().shownPpo2();
            if (heldPpo2 < 0.30f)
                FAIL("dive %d: the last believable ppO2 was %.2f bar — nothing to hold, the "
                     "window below cannot grade a hold", id, heldPpo2);
            pTrueAtOpen = rig.world().ppo2True();
            fastOpen = rig.deco().tissueBar(0);
            twinFastOpen = twin.deco().tissueBar(0);
            cnsOpen = rig.cnsPct();
            twinCnsOpen = twin.cnsPct();
            pulsesAtOpen = rig.pulseCount();
            twinPulsesAtOpen = twin.pulseCount();
            blindOpen = true;
        }
        if (frozen && !detected && tS - freezeAtS > kDetectDeadlineS)
            FAIL("dive %d: %.1f s after all three channels went still the vote is still alive "
                 "(act=%o, mode=%d) — the frozen-reading detector never fired",
                 id, tS - freezeAtS, (unsigned)rig.vote().activeMask, (int)rig.vote().mode);

        // ---- the hazard band: the fossil vote is still driving the valve --
        if (frozen && blindOpen == false && tS > freezeAtS) {
            const float p = rig.world().ppo2True();
            if (p > pTrueMaxInHazard) pTrueMaxInHazard = p;
            if (p > kHazardHighBar)
                FAIL("dive %d: the fossil vote drove the loop to %.2f bar in the %d cycles before "
                     "detection — the detector window must be short enough that a frozen vote "
                     "cannot enrich a loop into toxicity", id, p, hazardCycles);
            if (p < kHazardLowBar)
                FAIL("dive %d: the loop was already at %.2f bar %d cycles after the freeze — the "
                     "frozen values were fiction from the first instant, and some gate other "
                     "than the frozen detector should have caught this", id, p, hazardCycles);
        }

        if (blindOpen && !blindClosed && tS >= clearAtS) {
            // Act 4: the connector dries. Dither returns; each channel refutes
            // its own accusation by moving.
            World& w = rig.world();
            for (int c = 0; c < 3; c++) {
                w.cells[c].fault = CellFault::NONE;
                w.cells[c].noDither = false;
                w.cells[c].tauS = 0.6f;
            }
            fastClose = rig.deco().tissueBar(0);
            twinFastClose = twin.deco().tissueBar(0);
            cnsClose = rig.cnsPct();
            twinCnsClose = twin.cnsPct();
            ceilClose = rig.deco().ceilingM();
            twinCeilClose = twin.deco().ceilingM();
            pulsesInWindow = (int)(rig.pulseCount() - pulsesAtOpen);
            twinPulsesInWindow = (int)(twin.pulseCount() - twinPulsesAtOpen);
            blindClosed = true;
        }

        // ---- the rig and its twin, at the same true depth -----------------
        rig.world().setDepthM(dv.depth);
        twin.world().setDepthM(dv.depth);
        rig.tick();
        twin.tick();

        const uint16_t live = rig.alarmBits();
        const uint16_t logv = rig.alarmLogBits();
        alarmUnion |= live;
        const float sp = rig.spTarget();
        if (sp < kSpFloorBar || sp > kSpCeilBar)
            FAIL("dive %d: setpoint escaped to %.2f bar at t=%.0fs depth=%.1fm", id, sp, tS,
                 dv.depth);
        const float pTrue = rig.world().ppo2True();
        if (pTrue < kStarvationFloorBar)
            FAIL("dive %d: loop fell to %.2f bar at %.1f m (t=%.0fs alarms=0x%03x) — the "
                 "scenario drove the rig into a starvation the twin was not in", id, pTrue,
                 dv.depth, tS, alarmUnion);

        // ---- inside the no-vote window, cycle by cycle --------------------
        if (blindOpen && !blindClosed) {
            windowCycle++;
            if (pTrue < pTrueMinInWindow) pTrueMinInWindow = pTrue;
            if (rig.state().shownPpo2 < heldSeenMin) heldSeenMin = rig.state().shownPpo2;
            if (rig.state().shownPpo2 > heldSeenMax) heldSeenMax = rig.state().shownPpo2;
            if (rig.state().haveVote)
                FAIL("dive %d: the vote came back at t=%.0fs while all three channels are still "
                     "frozen — the bar leaked", id, tS);
            if (std::fabs(rig.state().shownPpo2 - heldPpo2) > 1e-6f)
                FAIL("dive %d: the published ppO2 moved to %.3f during a failed vote (held "
                     "%.3f) — 9002's hold must cover this path too", id, rig.state().shownPpo2,
                     heldPpo2);
            if (rig.vote().votedPpo2 != 0.0f)
                FAIL("dive %d: a failed vote reported %.2f bar to haveVote-gated consumers", id,
                     rig.vote().votedPpo2);
            if (live & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) {
                if (failAtCycle < 0) failAtCycle = windowCycle;
            }
            if (windowCycle * dtS >= kSensorFailDeadlineS && failAtCycle < 0)
                FAIL("dive %d: %.0f s into a FAILED vote with cellsSeen, BIT_SENSOR_FAIL had not "
                     "risen (alarms=0x%03x) — detection means shouting, not surviving",
                     id, windowCycle * dtS, live);
            if (!(logv & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
                FAIL("dive %d: the LOG view lost BIT_SENSOR_FAIL at t=%.0fs", id, tS);
            if (live & (uint16_t)(1u << alarms::BIT_LOW_PPO2)) lowDuring++;
            if (live & (uint16_t)(1u << alarms::BIT_HIGH_PPO2)) highDuring++;
            // The valve: shut. The held number must not drive injection.
            if (rig.pulseCount() != pulsesAtOpen)
                FAIL("dive %d: the valve pulsed at t=%.0fs with no believable ppO2 (held %.2f "
                     "vs setpoint %.2f)", id, tS, heldPpo2, sp);
        }
        if (blindClosed && !voteBack && rig.loop().haveVote()) {
            voteBack = true;
            voteBackS = tS;
        }
        if (voteBack && rig.pulseCount() > pulsesAtOpen && !pulsesResumed) pulsesResumed = true;

        // ---- the 30 s dive-file row, by the firmware's own fill -----------
        if (blindOpen && !blindClosed) blindCyclesInRow++;
        if (tS >= rowClockS) {
            const ctrl::State& st = rig.state();
            logfmt::RowInputs ri{};
            ri.nowMs = static_cast<uint32_t>(tS * 1000.0f);
            ri.diveStartMs = 0;
            ri.smoothedDepthM = st.depthSmoothedM;
            ri.haveVote = st.haveVote;
            ri.shownPpo2 = st.shownPpo2;
            for (int c = 0; c < 3; c++)
                ri.cellBar[c] = rig.cellStatus(static_cast<uint8_t>(c)).last;
            ri.setpointBar = st.setpointBar;
            ri.diveActive = true;
            ri.autoMode = st.autoMode;
            ri.alarmLogBits = st.alarms.logBits;
            const long long cnsX = numfmt::scaled(rig.cnsPct(), 0);
            ri.cnsPeakPct = static_cast<uint8_t>(cnsX > 254 ? 254 : cnsX);
            ri.vbat = VBAT_CELLS * 3.90f;
            ri.pulsesNow = rig.pulseCount();
            ri.pulsesPrev = rowPulseBase;
            // The published picture, computed exactly the way ControlTask
            // computes it (deco/view.h): the soak mirrors the PUBLICATION,
            // not the engine walk — since N8 the engine walk does not exist.
            ri.hasDecoPicture = true;
            deco::viewFull(ri.deco, rig.loop().deco(),
                           rig.loop().deco().depthToBar(ri.smoothedDepthM),
                           ri.haveVote, ri.shownPpo2);
            logfmt::fillSnapshot(row, ri);
            rowPulseBase = ri.pulsesNow;
            if (!blindOpen) rowHonest = row;
            // A row counts as a record OF the window when at least 6 of its
            // 30 s are inside it. The window here is 30 s and the row-clock
            // phase relative to detection is not controllable (9002 could
            // demand a wholly-blind row at 65 s; a window this short cannot),
            // but every cycle of this window shows the same held number and
            // the same marker, so a third-deep row records exactly what
            // 9002's wholly-blind rows record — and the marker assert below
            // still requires ALL counting rows to carry it.
            if (blindCyclesInRow >= 6 && blindOpen && !blindClosed) {
                rowsBlind++;
                rowBlind = row;
                if (row.alarmBits & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) rowsBlindMarker++;
            }
            rowClockS += 30.0f;
            blindCyclesInRow = 0;
        }

        // ---- the simulated diver -----------------------------------------
        const bool bits = live != 0;
        if (bits && alarmSinceS < 0.0f) alarmSinceS = tS;
        if (!bits) alarmSinceS = -1.0f;
        if (alarmSinceS >= 0.0f && tS - alarmSinceS >= ackDelayS) {
            rig.ackAlarms();
            alarmSinceS = -1.0f;
        }
        if (blindOpen && !blindClosed &&
            !(live & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)) &&
            (logv & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
            ackSilencedCycles++;

        if (dv.step(dtS, rig.deco(), tS)) finished = true;
    }
    rig.endDive();
    twin.endDive();

    // ---- grading ----------------------------------------------------------
    if (!frozen) FAIL("dive %d: the channels were never frozen", id);
    if (!detected) FAIL("dive %d: the vote never failed after the freeze", id);
    if (!blindClosed) FAIL("dive %d: the no-vote window never opened/closed", id);
    if (detectAtS - freezeAtS > kDetectDeadlineS)
        FAIL("dive %d: detection took %.1f s", id, detectAtS - freezeAtS);
    if (failAtCycle < 0) FAIL("dive %d: BIT_SENSOR_FAIL was never raised in the window", id);
    if (failAtCycle * dtS > kSensorFailDeadlineS)
        FAIL("dive %d: BIT_SENSOR_FAIL took %d cycles to follow the failed vote", id, failAtCycle);
    if (heldSeenMax - heldSeenMin > 1e-6f)
        FAIL("dive %d: the published ppO2 drifted across the window (%.3f -> %.3f)", id,
             heldSeenMin, heldSeenMax);
    if (lowDuring || highDuring)
        FAIL("dive %d: a level alarm (LOW %d, HIGH %d cycles) stood on a ppO2 that was no "
             "longer a measurement", id, lowDuring, highDuring);
    if (pulsesInWindow != 0)
        FAIL("dive %d: %d injection pulses with no believable ppO2 (the twin pulsed %d at the "
             "same depth)", id, pulsesInWindow, twinPulsesInWindow);
    if (twinPulsesInWindow <= 0)
        FAIL("dive %d: the twin pulsed %d times across the window, so the valve assert above "
             "distinguishes nothing", id, twinPulsesInWindow);
    if (pTrueAtOpen - pTrueMinInWindow < kPremiseLoopFallBar)
        FAIL("dive %d: the loop only fell %.2f bar during %.0f s of no injection (%.2f -> %.2f) "
             "— the deco direction asserts would prove nothing", id,
             pTrueAtOpen - pTrueMinInWindow, kWindowS, pTrueAtOpen, pTrueMinInWindow);

    // Tissues: conservative no-vote loading (9002's decision, same rule, this
    // path too). The held ppO2 would have UNDER-recorded.
    const double gainRig = fastClose - fastOpen, gainTwin = twinFastClose - twinFastOpen;
    if (!(gainRig >= gainTwin + kMinTissueGainBar))
        FAIL("dive %d: across the %.0f s no-vote window the fast tissue gained %.3f bar against "
             "the twin's %.3f — the tissues are being fed the held ppO2", id, kWindowS, gainRig,
             gainTwin);
    if (!(ceilClose >= twinCeilClose))
        FAIL("dive %d: ceiling %.1f m vs twin %.1f m after the window", id, ceilClose, twinCeilClose);

    // CNS frozen across the no-vote window (the documented residual, pinned as
    // in 9002), with the twin proving the dive itself was running.
    if (!(std::fabs(cnsClose - cnsOpen) < 1e-9))
        FAIL("dive %d: the oxygen clock advanced %.4f %% during a failed vote", id,
             cnsClose - cnsOpen);
    if (!(twinCnsClose > twinCnsOpen + 0.01))
        FAIL("dive %d: the twin's CNS moved only %.4f %% over the same span", id,
             twinCnsClose - twinCnsOpen);

    // The dive file: rows from the blind window carry the held number WITH the
    // marker (the 9002 machinery, graded here on the detector-triggered path).
    if (rowsBlind == 0)
        FAIL("dive %d: no dive-file row spent at least 6 s inside the %.0f s no-vote window", id,
             kWindowS);
    const int32_t wantX = static_cast<int32_t>(numfmt::scaled(heldPpo2, 3));
    if (std::abs(rowBlind.votedX1000 - wantX) > 1)
        FAIL("dive %d: the blind row recorded %.3f ppO2, not the held %.3f", id,
             (float)rowBlind.votedX1000 / 1000.0f, heldPpo2);
    if (!(rowBlind.alarmBits & (uint16_t)(1u << alarms::BIT_SENSOR_FAIL)))
        FAIL("dive %d: the blind row carries a held ppO2 unmarked (0x%03x)", id, rowBlind.alarmBits);
    if (rowBlind.ndlMin != deco::kNDL_UNKNOWN)
        FAIL("dive %d: the blind row reports NDL %d instead of the unknown sentinel (-1, "
             "one pair file+wire since N5)", id, (int)rowBlind.ndlMin);
    if (rowsBlindMarker != rowsBlind)
        FAIL("dive %d: only %d of %d blind rows carried BIT_SENSOR_FAIL", id, rowsBlindMarker,
             rowsBlind);
    if (ackSilencedCycles == 0)
        FAIL("dive %d: the ACK never separated the live and log views in the window", id);

    // ---- act 4: the thaw. Cells refute their OWN accusations. -------------
    if (!voteBack)
        FAIL("dive %d: the vote never came back after the channels started moving again", id);
    if (voteBackS - clearAtS > kRecoverDeadlineS)
        FAIL("dive %d: the vote took %.1f s to return after the thaw (deadline %.1f s)", id,
             voteBackS - clearAtS, kRecoverDeadlineS);
    if (rig.vote().activeMask != 0b111)
        FAIL("dive %d: after the thaw the vote is act=%o — a refuted cell stayed out", id,
             (unsigned)rig.vote().activeMask);
    for (int c = 0; c < 3; c++)
        if (rig.cellBarReason((uint8_t)c) != voting::Engine::BAR_NONE)
            FAIL("dive %d: ch%d still barred with reason %d after moving for %.0f s — the frozen "
                 "bar outlived its own refutation", id, c, rig.cellBarReason((uint8_t)c),
                 tS - clearAtS);
    if (std::fabs(rig.state().shownPpo2 - rig.vote().votedPpo2) > 1e-6f)
        FAIL("dive %d: with a vote running the shown ppO2 (%.3f) is not the vote (%.3f)", id,
             rig.state().shownPpo2, rig.vote().votedPpo2);
    if (!pulsesResumed)
        FAIL("dive %d: injection never resumed after the vote did (loop %.2f, sp %.2f)", id,
             rig.world().ppo2True(), rig.spTarget());
    // The twin ran the same profile for the whole dive and never lost a cell:
    // the detector's window (700 s of quiet hold on this dive) is also the
    // false-positive proof.
    if (twin.vote().activeMask != 0b111)
        FAIL("dive %d: the twin ended the dive with act=%o", id, (unsigned)twin.vote().activeMask);

    std::printf("dive %d  frozen-channels  %5.0f m  quiet %3.0f s -> frozen %3.0f s -> detected "
                "%3.0f s (%.0f s, all ch STUCK) -> blind %3.0f s  held ppO2 %.3f (loop fell to "
                "%.2f, hazard peak %.2f)  pulses %d vs twin %d  CNS %+.2f%% vs twin %+.2f%%  "
                "fast tissue %+.2f vs twin %+.2f  ceil %.1f vs twin %.1f m  rows %d (all marked, "
                "NDL %d)  no-vote cycles %d  thaw->vote %.1f s  alarms 0x%03x\n",
                id, kBottomM, kFreezeAtS, freezeAtS, detectAtS, detectAtS - freezeAtS, kWindowS,
                heldPpo2, pTrueMinInWindow, pTrueMaxInHazard, pulsesInWindow, twinPulsesInWindow,
                cnsClose - cnsOpen, twinCnsClose - twinCnsOpen, gainRig, gainTwin, ceilClose,
                twinCeilClose, rowsBlind, (int)rowBlind.ndlMin, windowCycle,
                voteBackS - clearAtS, alarmUnion);
    std::printf("dive %d  detector-view  3/3 channels accused STUCK within %.0f s of the freeze, "
                "3/3 refuted and voting within %.1f s of the thaw; twin: 0 accusations in %d "
                "cycles of quiet hold  (FAILED->SENSOR_FAIL %d cycle, valve shut %d cycles, "
                "rows marked, tissues conservative)\n",
                id, detectAtS - freezeAtS, voteBackS - clearAtS, (int)(tS / dtS), failAtCycle,
                windowCycle);
}

// ---------------------------------------------------------------------------
// Dive 9004 — dive continuity (review H1): "the one gap that can kill without
// any instrument failing".
//
// The scenario in one sentence: dive 1 ends with an obligation, the diver
// surfaces, and dive 2 starts 60 minutes later — and dive 2 must show a ceiling
// the moment it starts. It used not to. startDive() called deco_.reset(), so the
// second dive was computed as if the first had never happened: ceiling 0, NDL
// wide open, no deco schedule, and every one of those numbers displayed as a
// fact. No sensor had failed, no alarm was due, and the controller was the only
// thing in the system that knew the difference.
//
// What the rig runs here is the shipping logic with the file calls removed:
// logfmt::ContinuitySlots is the real two-slot store (same CRC gate, same
// "write the OLDER slot" rule, same sequence-off-the-slots), the cadence is
// main.cpp's (dive-end record, 60 s grid during the dive, 1 Hz seed refresh at
// the surface), and the off-gassing across the interval is the deco engine's own
// surface integration. What is simulated is LittleFS and the wall clock
// (Rig::epoch(), the stand-in for comms::clockEpoch()).
//
// The twin is the point of the whole dive: an identical rig with continuity OFF,
// the same profile, the same diver. The difference between the two at the start
// of dive 2 is the size of the bug, in metres and minutes, and it is measured
// rather than asserted against a number this file made up.
// ---------------------------------------------------------------------------
void runContinuityPair() {
    const int id = 9004;
    constexpr float kBottomM = 40.0f;
    // Chosen, not tuned. 30 minutes at this depth leaves an hour-long surface
    // interval that clears the obligation entirely, and then dive 2's ceiling is
    // honestly zero with or without continuity — the scenario would prove
    // nothing. An hour on the bottom leaves a real obligation an hour later: the
    // witness line shows the entry ceiling in metres and the no-deco time at 20 m
    // (0 min against the twin's 39), which is the size of what the reset took.
    constexpr float kBottomS = 3600.0f;
    constexpr float kSurfaceS = 3600.0f;   // the reviewer's hour out of the water
    constexpr float kDive2DepthM = 20.0f;
    constexpr float kDive2S = 300.0f;
    const float dtS = Rig::kTickMs / 1000.0f;

    // One scripted dive, ended the way a diver ends one when he is in a hurry:
    // down, hold, straight back up through the ceiling, log closed with the
    // obligation still outstanding. The ascent deliberately does NOT honour the
    // ceiling — that is the case the record exists for. A diver who decompressed
    // all the way up would not need continuity, and a scenario built on him would
    // prove nothing.
    struct Profiler {
        Rig& rig;
        float dtS;
        float depth = 0.0f;
        void moveTo(float target, float rateMps) {
            while (std::fabs(depth - target) > 1e-3f) {
                depth = (depth < target) ? std::min(target, depth + rateMps * dtS)
                                         : std::max(target, depth - rateMps * dtS);
                rig.world().setDepthM(depth);
                rig.tick();
            }
        }
        void hold(float seconds) {
            const int n = static_cast<int>(seconds / dtS);
            for (int i = 0; i < n; i++) { rig.world().setDepthM(depth); rig.tick(); }
        }
    };
    auto diveOne = [&](Rig& rig) {
        prediveHealthy(rig);
        if (!rig.predivePassed()) FAIL("dive %d: pre-dive failed on a clean rig", id);
        if (!rig.requestAuto(true)) FAIL("dive %d: AUTO refused after a passed pre-dive", id);
        rig.startDive();
        Profiler p{rig, dtS};
        p.moveTo(kBottomM, 0.25f);
        p.hold(kBottomS);
        p.moveTo(0.0f, 0.15f);        // through the ceiling: the diver chose this
        rig.endDive();
        p.hold(5.0f);                 // the dive-end edge lands on the next tick
    };

    // ---- the subject: continuity on -----------------------------------------
    RigOpts on = soakOpts();
    on.continuity = true;
    on.bootEpoch = 1800000000u;         // a board whose clock was set at the surface
    Rig rig(on);
    diveOne(rig);

    const float endCeilM = static_cast<float>(rig.deco().ceilingM());
    const double endFast = rig.deco().tissueBar(0);            // 5 min half-time
    const double endMid = rig.deco().tissueBar(7);             // 77 min half-time
    const double endSlow = rig.deco().tissueBar(deco::Zhl16Ccr::NCOMP - 1);  // 635 min
    const float endCns = rig.cnsPct();
    // The premise, checked not assumed: if the first dive left no obligation,
    // every number below is a witness about nothing.
    if (endCeilM <= 0.3f)
        FAIL("dive %d: dive 1 ended with a ceiling of %.1f m — the scenario needs an "
             "obligated diver, this profile is not one", id, endCeilM);

    // What the store actually holds after that dive: the dive-END record, in the
    // slot the writes chose (the 60 s grid wrote ~30 records into two slots, so
    // the picture is a ping-pong by now, and the newest one must still decode).
    if (!rig.contHasRecord()) FAIL("dive %d: no continuity record after dive 1", id);
    if (rig.contCorrupt()) FAIL("dive %d: the continuity store reports its own record corrupt", id);
    logfmt::ContinuityRecord stored{};
    TEST_ASSERT_TRUE(rig.contSlots().readNewest(stored));
    TEST_ASSERT_EQUAL_UINT8(logfmt::CONT_KIND_DIVE_END, stored.kind);

    // The hour out of the water. The deco engine is not integrated outside a dive
    // (loop.cpp step 8 gates stepTissues on diveActive), so these ticks move the
    // CLOCK, not the tissues: the off-gassing for this hour is credited once, at
    // the dive start, by the engine's own surface integration. If that gate ever
    // moves, this scenario becomes a double-credit and the asserts below are the
    // thing that says so.
    {
        Profiler p{rig, dtS};
        p.hold(kSurfaceS);
    }
    const uint32_t seqBeforeDive2 = stored.seq;

    // ---- dive 2: the witness -----------------------------------------------
    rig.startDive();
    rig.world().setDepthM(0.0f);
    rig.tick();
    const ctrl::Loop::DiveSeed seed2 = rig.contLastSeed();
    const float entryCeilM = static_cast<float>(rig.deco().ceilingM());
    const double entryFast = rig.deco().tissueBar(0);
    const double entryMid = rig.deco().tissueBar(7);
    const double entrySlow = rig.deco().tissueBar(deco::Zhl16Ccr::NCOMP - 1);
    const float entryCns = rig.cnsPct();
    const float entryNdl20 = static_cast<float>(
        rig.deco().ndlMinutes(rig.deco().depthToBar(kDive2DepthM), 0.9f));

    if (!seed2.applied)
        FAIL("dive %d: dive 2 did not seed from the stored record (loss reason %u) — the "
             "diver who surfaced obligated was handed a fresh dive", id,
             static_cast<unsigned>(rig.contLastLoss()));
    TEST_ASSERT_EQUAL_UINT8(logfmt::CONT_KIND_DIVE_END, seed2.kind);
    TEST_ASSERT_EQUAL_UINT32(seqBeforeDive2, seed2.recordSeq);
    // The credited interval is the hour, give or take the 1 Hz seed refresh — and
    // it is the number the console line and the EVT_CONT_RESTORED record carry,
    // so the soak is asserting on what the machine SAYS it credited.
    if (seed2.creditS < (uint32_t)kSurfaceS - 10u || seed2.creditS > (uint32_t)kSurfaceS + 10u)
        FAIL("dive %d: dive 2 credited %u s of surface interval, expected about %u s",
             id, (unsigned)seed2.creditS, (unsigned)kSurfaceS);
    // The reviewer's requirement, as an assert: a ceiling AT ENTRY, before any
    // descent, from the tissues the previous dive left.
    if (entryCeilM <= 0.0f)
        FAIL("dive %d: dive 2 shows ceiling %.1f m at entry with %.2f bar in the fast "
             "tissue — the continuity this file exists for is not reaching the deco picture",
             id, entryCeilM, entryFast);
    // Which tissues carry the debt, and why the bug was invisible at the fast end:
    // an hour is twelve half-times for the 5 min compartment, so it is home at the
    // surface asymptote whether or not continuity exists. The debt an hour after
    // a 40 m / 50 min dive lives in the slow compartments — the 77 min one is still
    // near double its resting load, and the 635 min one has barely begun to come
    // down. Those are the numbers a repetitive dive is planned against, and the
    // ones a reset throws away.
    const double resting = rig.deco().surfacePpN2Bar();
    if (!(entryFast < endFast - 1.0))
        FAIL("dive %d: the 5 min tissue did not off-gas across the interval (%.3f -> %.3f)",
             id, endFast, entryFast);
    if (entryMid <= resting + 0.5)
        FAIL("dive %d: dive 2 started its 77 min tissue at %.3f bar, i.e. from clean tissues "
             "(resting is %.3f)", id, entryMid, resting);
    if (!(entryMid < endMid && entryMid > resting))
        FAIL("dive %d: the 77 min tissue did not move into the surface interval (%.3f -> %.3f)",
             id, endMid, entryMid);
    if (!(entrySlow < endSlow && endSlow - entrySlow < 0.1))
        FAIL("dive %d: the 635 min tissue moved by %.3f bar in an hour — it should have "
             "barely begun", id, endSlow - entrySlow);
    // CNS: carried, not decayed. This engine has no oxygen-clock decay term, so
    // continuity carries the clock exactly; an hour out of the water buys nothing.
    // The tolerance is half the storage quantum (CNS travels as tenths of a
    // percent), plus one 100 ms tick of the new dive — not a fudge factor: a
    // difference LARGER than that means the clock was recomputed somewhere, and
    // that is exactly the drift this assert is for.
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.06, endCns, entryCns,
                                     "CNS changed across the surface interval");

    // A short second dive, so the NDL and ceiling are witnessed in a dive rather
    // than only at entry.
    {
        Profiler p{rig, dtS};
        p.moveTo(kDive2DepthM, 0.25f);
        p.hold(kDive2S);
    }
    const float dive2CeilM = static_cast<float>(rig.deco().ceilingM());
    const float dive2Ndl20 = static_cast<float>(
        rig.deco().ndlMinutes(rig.deco().depthToBar(kDive2DepthM), 0.9f));
    rig.endDive();
    rig.tick();
    // Dive 2's own end record has to be on the partition now, newer than dive 1's:
    // the debt is cumulative across the day, which is the whole point.
    logfmt::ContinuityRecord afterDive2{};
    TEST_ASSERT_TRUE(rig.contSlots().readNewest(afterDive2));
    if (afterDive2.seq <= seqBeforeDive2)
        FAIL("dive %d: dive 2 left no newer continuity record (seq %u after %u)", id,
             (unsigned)afterDive2.seq, (unsigned)seqBeforeDive2);

    // ---- the honest twin: identical rig, continuity OFF --------------------
    // This is the pre-H1 firmware, running now, on the same profile. Its entry
    // numbers are the size of the bug.
    RigOpts off = soakOpts();
    off.continuity = false;
    off.bootEpoch = on.bootEpoch;
    Rig twin(off);
    diveOne(twin);
    const double twinEndFast = twin.deco().tissueBar(0);
    if (std::fabs(twinEndFast - endFast) > 0.02)
        FAIL("dive %d: twin and subject diverged during dive 1 (%.3f vs %.3f bar) — dive 2 "
             "is not a controlled comparison", id, twinEndFast, endFast);
    {
        Profiler p{twin, dtS};
        p.hold(kSurfaceS);
    }
    twin.startDive();
    twin.world().setDepthM(0.0f);
    twin.tick();
    const float twinEntryCeilM = static_cast<float>(twin.deco().ceilingM());
    const double twinEntryFast = twin.deco().tissueBar(0);
    const double twinEntryMid = twin.deco().tissueBar(7);
    const float twinEntryNdl20 = static_cast<float>(
        twin.deco().ndlMinutes(twin.deco().depthToBar(kDive2DepthM), 0.9f));
    if (twinEntryCeilM > 0.0f)
        FAIL("dive %d: the continuity-OFF twin shows a ceiling of %.1f m at entry — this "
             "rig is not the pre-H1 behaviour it is being compared against", id, twinEntryCeilM);
    if (std::fabs(twinEntryMid - twin.deco().surfacePpN2Bar()) > 0.01)
        FAIL("dive %d: the twin's entry 77 min tissue is %.3f bar, not the resting load — the "
             "twin seeded when it should not have", id, twinEntryMid);
    if (!(entryCeilM > twinEntryCeilM))
        FAIL("dive %d: dive 2 ceiling %.1f m is not deeper than the twin's %.1f m", id,
             entryCeilM, twinEntryCeilM);
    if (!(entryNdl20 < twinEntryNdl20))
        FAIL("dive %d: dive 2 NDL at 20 m is %.0f min against the twin's %.0f min", id,
             entryNdl20, twinEntryNdl20);

    // ---- no credit: the same record, a clock that cannot establish the hour --
    // Three real paths do this (no clock on either side, a record dated after the
    // clock, a mid-dive snapshot that cannot say when the diver surfaced), and the
    // engine's answer is the same for all of them: carry the tissues, credit
    // nothing. The witness here is the clock one, and the direction is the point:
    // a deeper ceiling than the truth, never a shallower one.
    RigOpts nc = on;
    Rig noclock(nc);
    diveOne(noclock);
    const double ncEndFast = noclock.deco().tissueBar(0);
    const double ncEndMid = noclock.deco().tissueBar(7);
    {
        Profiler p{noclock, dtS};
        p.hold(kSurfaceS);
    }
    noclock.setClockLost(true);          // the epoch died with the previous boot
    {
        Profiler p{noclock, dtS};
        p.hold(2.0f);                    // the 1 Hz refresh republishes the seed, without it
    }
    noclock.startDive();
    noclock.world().setDepthM(0.0f);
    noclock.tick();
    const ctrl::Loop::DiveSeed ncSeed = noclock.contLastSeed();
    const float ncCeilM = static_cast<float>(noclock.deco().ceilingM());
    const double ncFast = noclock.deco().tissueBar(0);
    const double ncMid = noclock.deco().tissueBar(7);
    TEST_ASSERT_TRUE_MESSAGE(ncSeed.applied,
                             "the no-credit rig must still seed: tissues carried, only the "
                             "interval refused");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0u, ncSeed.creditS,
                                     "an interval that cannot be established must credit zero");
    TEST_ASSERT_EQUAL_UINT8(logfmt::INT_NO_NOW_CLOCK, ncSeed.creditWhy);
    // The claim "no interval was credited" is checked on the state the chain
    // CONSUMED, not on the state after dive 2 has been running: one tick of a new
    // dive at the surface moves a compartment by ~1e-5 bar all by itself, and that
    // is the dive's own integration, not credit. Against the stored record, the
    // seed's tissues are the dive-END loadings to within the storage quantum.
    if (std::fabs(ncSeed.st.pt[7] - ncEndMid) > 2e-4)
        FAIL("dive %d: the no-credit rig's seed moved the 77 min tissue from %.4f to %.4f bar "
             "— refusing an interval has to mean refusing it", id, ncEndMid, ncSeed.st.pt[7]);
    if (std::fabs(ncMid - ncSeed.st.pt[7]) > 1e-3)
        FAIL("dive %d: the no-credit rig's engine disagrees with the seed it was given", id);
    if (!(ncCeilM >= entryCeilM))
        FAIL("dive %d: refusing the interval made the ceiling SHALLOWER (%.1f m vs %.1f m) — "
             "the conservative direction is the other way round", id, ncCeilM, entryCeilM);
    if (ncFast + 1e-12 < entryFast)
        FAIL("dive %d: the no-credit rig is LESS loaded than the credited one (%.3f vs %.3f)",
             id, ncFast, entryFast);

    // ---- the record is gone: reset, and say so -----------------------------
    // A torn write or a page that stopped holding its CRC. The dive has to start
    // from clean tissues (today's behaviour, and safe as a fallback) and the
    // machine has to be able to name what happened: "no record" and "a record we
    // cannot read" are different sentences, and this is the case where the diver
    // was owed a ceiling and was given none.
    RigOpts bad = on;
    Rig corrupt(bad);
    diveOne(corrupt);
    // First the half-good news, which the two-slot layout is the whole reason for:
    // damaging the NEWEST record costs a snapshot, not the picture. The dive-END
    // record and the last grid snapshot are in different slots, so the older one
    // still decodes and dive 2 is still seeded — a torn write cannot take away the
    // only copy of a diver's debt.
    logfmt::ContinuityRecord newestBeforeDamage{};
    TEST_ASSERT_TRUE(corrupt.contSlots().readNewest(newestBeforeDamage));
    const uint32_t seqBeforeDamage = newestBeforeDamage.seq;
    corrupt.contCorruptNewest();
    if (!corrupt.contHasRecord())
        FAIL("dive %d: damaging one slot left the store with no record at all — the two-slot "
             "layout is supposed to cost a snapshot, not the picture", id);
    {
        Profiler p{corrupt, dtS};
        p.hold(2.0f);   // the 1 Hz refresh re-publishes: dive 2 seeds from the OTHER slot
    }
    corrupt.startDive();
    corrupt.world().setDepthM(0.0f);
    corrupt.tick();
    const ctrl::Loop::DiveSeed halfSeed = corrupt.contLastSeed();
    const float halfCeilM = static_cast<float>(corrupt.deco().ceilingM());
    TEST_ASSERT_TRUE_MESSAGE(halfSeed.applied,
                             "one damaged slot must still seed the dive from the other one");
    if (halfSeed.recordSeq >= seqBeforeDamage || halfSeed.recordSeq == 0u)
        FAIL("dive %d: after one slot was damaged the dive seeded from record %u, expected the "
             "older survivor (the damaged one was %u)", id, (unsigned)halfSeed.recordSeq,
             (unsigned)seqBeforeDamage);
    // The survivor is a mid-dive grid snapshot, not a dive-end record — and the
    // mid-dive rule says it credits no interval, because it cannot say when the
    // diver surfaced. So this dive is seeded, conservatively: the ceiling it shows
    // is the one from tissues that never got their hour.
    TEST_ASSERT_EQUAL_UINT8(logfmt::CONT_KIND_PERIODIC, halfSeed.kind);
    TEST_ASSERT_EQUAL_UINT32(0u, halfSeed.creditS);
    if (halfCeilM <= 0.0f)
        FAIL("dive %d: one damaged slot cost the diver his ceiling (%.1f m)", id, halfCeilM);
    corrupt.endDive();
    {
        Profiler p{corrupt, dtS};
        p.hold(5.0f);
    }

    // Now the case that is NOT survivable: every slot damaged (a dead page, a
    // wiped partition). The dive has to start from clean tissues AND SAY SO.
    corrupt.contCorruptAll();
    if (corrupt.contHasRecord()) FAIL("dive %d: the damaged record still decodes", id);
    if (!corrupt.contCorrupt()) FAIL("dive %d: a corrupt store is not reporting itself corrupt", id);
    {
        Profiler p{corrupt, dtS};
        p.hold(kSurfaceS);
    }
    corrupt.startDive();
    corrupt.world().setDepthM(0.0f);
    corrupt.tick();
    const float corruptCeilM = static_cast<float>(corrupt.deco().ceilingM());
    TEST_ASSERT_FALSE_MESSAGE(corrupt.contLastSeed().applied,
                              "a corrupt record must not seed a dive");
    TEST_ASSERT_EQUAL_UINT8(logfmt::CONT_LOSS_BAD, static_cast<uint8_t>(corrupt.contLastLoss()));
    if (corruptCeilM > 0.0f)
        FAIL("dive %d: a dive with no record shows a ceiling of %.1f m", id, corruptCeilM);

    // Two witness lines, in the soak's own form: the pair (subject vs honest twin)
    // and the conservative direction of every no-credit path.
    std::printf("dive %d  continuity  %4.0f m/%2.0f min then %3.0f min surface  "
                "dive 1 ends obligated ceil %4.1f m  77 min tissue %5.2f -> %5.2f bar  "
                "dive 2 entry ceil %4.1f m vs twin %4.1f m  NDL@20m %3.0f vs twin %3.0f min  "
                "credited %u s  CNS %4.1f%% carried  seq %u->%u\n",
                id, kBottomM, kBottomS / 60.0f, kSurfaceS / 60.0f, endCeilM, endMid, entryMid,
                entryCeilM, twinEntryCeilM, entryNdl20, twinEntryNdl20,
                (unsigned)seed2.creditS, entryCns, (unsigned)seqBeforeDive2,
                (unsigned)afterDive2.seq);
    std::printf("dive %d  continuity-views  no-credit: seeded, 0 s credited, ceil %4.1f m (never "
                "shallower than the credited %4.1f)  corrupt record: reset, loss=%s, ceil %4.1f m  "
                "dive 2 in-dive ceil %4.1f m, NDL %3.0f min\n",
                id, ncCeilM, entryCeilM, logfmt::contLossName(corrupt.contLastLoss()),
                corruptCeilM, dive2CeilM, dive2Ndl20);
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

// The oxygen leg (finding N2): a flooded harness that leaves the chain with no
// ppO2 it is willing to believe, and the held-and-marked number it shows instead.
void test_wet_connector_all_three_cells_dead() { runWetConnectorDive(); }

void test_all_three_channels_freeze_plausible() { runFrozenChannelDive(); }

// Dive continuity (review H1): dive 1 ends obligated, an hour out of the water,
// dive 2 must show a ceiling at entry — against an identical rig with continuity
// off, which is the size of the bug in metres. See runContinuityPair.
void test_dive_continuity_repetitive_dive() { runContinuityPair(); }

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_soak_dives_01_13);
    RUN_TEST(test_soak_dives_14_25);
    RUN_TEST(test_soak_dives_26_38);
    RUN_TEST(test_soak_dives_39_50);
    RUN_TEST(test_depth_bus_dies_and_returns_mid_dive);
    RUN_TEST(test_wet_connector_all_three_cells_dead);
    RUN_TEST(test_all_three_channels_freeze_plausible);
    RUN_TEST(test_dive_continuity_repetitive_dive);
    return UNITY_END();
}
