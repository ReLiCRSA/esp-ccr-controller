// ============================================================================
// test_controlplane — the ControlTask call graph, measured and pinned (H2).
//
// The reviewer's H2 claim had three parts, and each gets a case here that
// reddens if the fix regresses:
//   1. "unbounded schedule() math ... on the control task" — MEASURED, not
//      argued: an adversarial-but-legal tissue load (deep, all 16 compartments
//      loaded, long deco with capped holds) runs the exact calls ControlTask
//      makes on the 30 s grid (fillSnapshot -> ndlMinutes + schedule), and the
//      worst case is pinned by a GENEROUS budget. This test cannot tell you
//      the ESP32 number — that is what HB `worst=` is for — its job is the
//      complexity class: if schedule() ever becomes unbounded or worse than
//      its documented quadratic-in-depth shape, a case here goes red.
//   2. "portMAX_DELAY log-mutex waits inside ControlTask" — pinned two ways:
//      the GridGate policy test (deferral, exact lost-row counting, original
//      timestamps preserved, flush-at-close not miscounted) and a SOURCE SCAN
//      of main.cpp's controlTask body plus session_wiring.cpp's control-task
//      entry points: no portMAX_DELAY anywhere on that path, and the unbounded
//      legacy appends must not appear in the control task's body at all.
//   3. "the max-on fuse evaluated only inside cycle()" — the pin-fuse policy
//      test: the fuse condition is a function of the WRITE TIMESTAMPS alone,
//      provable by a loop that drives the pin and never calls cycle(); and
//      the latch refuses any later re-assertion. If the fuse moves back into
//      cycle(), this file's drive-loop case no longer describes what ships —
//      the scan test and hw_accept's SOLFUSE bench check are the enforcement.
// ============================================================================
#include <unity.h>

#include <chrono>
#include <cstdio>
#include <string>

#include "deco/zhl16c.h"
#include "log/rowfill.h"
#include "log/rowgate.h"
#include "util/pinfuse.h"

namespace {

// ---------------------------------------------------------------------------
// (1) Worst-case deco math, measured. Adversarial-but-legal: a fast descent
// to depth, a long bottom at a deep setpoint, then the full row fill the 30 s
// grid performs at the deepest legal ambient this sensor can report.
// ---------------------------------------------------------------------------
deco::Zhl16Ccr loadedEngine(double depthM, double bottomMin, double ppO2) {
    deco::Config c;  // shipping defaults: gf 30/85, 6 m/min, 3 m grid
    deco::Zhl16Ccr z(c);
    double d = 0.0;
    for (; d < depthM; d += 20.0 / 60.0)  // 20 m/min descent, 1 s steps
        z.stepTissues(1.0, z.depthToBar(d), ppO2);
    const double pB = z.depthToBar(depthM);
    for (double m = 0.0; m < bottomMin; m += 1.0) z.stepTissues(60.0, pB, ppO2);
    return z;
}

// Worst fill time in us; when outBest is given, also reports the BEST (fastest)
// of the runs — best-case timing is the stable complexity measure, worst-case
// is what a loaded CI box's scheduler noise moves around.
double worstFillUs(deco::Zhl16Ccr& z, double depthM, double* outBest = nullptr) {
    logfmt::RowInputs ri{};
    ri.nowMs = 0; ri.diveStartMs = 0; ri.smoothedDepthM = depthM;
    ri.haveVote = true; ri.shownPpo2 = 1.30f; ri.setpointBar = 1.2f;
    ri.diveActive = true; ri.autoMode = true; ri.deco = &z;
    double worst = 0.0, best = 1e18;
    for (int r = 0; r < 20; ++r) {
        logfmt::Snapshot sn{};
        const auto t0 = std::chrono::steady_clock::now();
        logfmt::fillSnapshot(sn, ri);
        const auto t1 = std::chrono::steady_clock::now();
        const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        if (us > worst) worst = us;
        if (us < best) best = us;
    }
    if (outBest) *outBest = best;
    return worst;
}

void test_worst_case_row_fill_fits_the_control_budget() {
    // The deepest load the hardware can legally produce: LPS28DFW bottoms at
    // 250 mbar; ~148 m of sea water. Loaded at that depth, held 60 min.
    deco::Zhl16Ccr z = loadedEngine(148.0, 60.0, 1.30);
    const double us = worstFillUs(z, 148.0);
    printf("  controlplane: worst-case fill (148 m, capped holds) = %.0f us on host\n", us);
    // Budget: 25 ms — a ~50x factor over the measured host number (the native
    // env builds -O0; the ESP32 pays its soft-double factor separately, and
    // HB `worst=` measures that on real metal). The number is deliberately
    // loose: this assertion exists to catch schedule() GROWING — unbounded
    // loops, per-cycle recomputation, a new simulation dimension — not to
    // pin microseconds. Any jump of two orders of magnitude reddens it.
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(25000.0, us,
        "worst-case ControlTask row fill blew the pinned budget: the deco "
        "recompute is due-driven (30 s grid) and quadratic-in-depth by design; "
        "this number growing means that shape changed");
}

void test_fill_cost_grows_with_depth_not_worse_than_quadratic() {
    // The documented shape: quadratic in bottom depth (grid steps x per-step
    // hold time, both proportional to depth). Doubling the legal depth may
    // therefore cost ~4x, never 8x+. This is the case that reddens if the
    // walk ever becomes cubic or the caps stop holding.
    //
    // The ratio compares BEST-of-20 times, not worst: a scheduler hiccup on
    // a loaded build box inflates one shallow sample enough to fake a
    // super-quadratic ratio (seen in mutation testing: a green tree read
    // 6.6x purely from noise). Best-case time is the machine-independent
    // measure of how much WORK the walk does, which is what this case is
    // about. The budget test above keeps the worst-case number, because
    // ControlTask pays the worst beat, not the average one.
    double sa = 0.0, sb = 0.0;
    deco::Zhl16Ccr shallow = loadedEngine(45.0, 90.0, 1.30);
    deco::Zhl16Ccr deep = loadedEngine(90.0, 90.0, 1.30);
    const double a = worstFillUs(shallow, 45.0, &sa);
    const double b = worstFillUs(deep, 90.0, &sb);
    printf("  controlplane: fill 45 m = %.0f us best / %.0f us worst, "
           "90 m = %.0f us best / %.0f us worst, best-ratio %.2f (quadratic => ~4)\n",
           sa, a, sb, b, sa > 0 ? sb / sa : 0.0);
    TEST_ASSERT_TRUE_MESSAGE(sa > 0.0, "the shallow case measured no time at all: timer broken");
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(sa * 6.0, sb,
        "row-fill cost grew faster than the documented quadratic-in-depth shape");
}

void test_capped_schedule_reports_itself() {
    // The bound is only honest if hitting it is VISIBLE: a schedule that ran
    // out of its guard or hold caps says capped, and the diver never silently
    // gets half a plan.
    deco::Zhl16Ccr z = loadedEngine(148.0, 120.0, 1.30);
    const auto s = z.schedule(z.depthToBar(148.0));
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, s.capped,
        "the extreme schedule did not mark itself capped: an incomplete deco "
        "plan would read as complete");
    TEST_ASSERT_TRUE(s.count > 0);
}

// ---------------------------------------------------------------------------
// (2) GridGate: deferral instead of stalling, and a lost row counted out loud.
// ---------------------------------------------------------------------------
void test_gate_defers_and_retries_a_busy_append() {
    logfmt::GridGate gate;
    int written = 0;
    gate.setAppender([&](uint32_t budgetMs) {
        if (budgetMs == 0u) return logfmt::GridGate::BUSY;  // try-lock keeps failing
        if (budgetMs == 3u) { written++; return logfmt::GridGate::WRITTEN; }
        return logfmt::GridGate::BUSY;
    });
    gate.setFirstAttemptBudgetMs(3u);
    logfmt::Snapshot row{}; row.tSec = 90;
    gate.post(90000u, row);
    TEST_ASSERT_EQUAL_INT(1, written);
    TEST_ASSERT_EQUAL_UINT32(0, gate.lostRows());
    TEST_ASSERT_FALSE(gate.pending());
}

void test_busy_row_is_kept_written_once_and_never_double_counted() {
    logfmt::GridGate gate;
    int attempts = 0, grantedAt = -1;
    logfmt::Snapshot* writtenRow = nullptr;
    gate.setAppender([&](uint32_t) {
        attempts++;
        if (attempts == grantedAt) {
            if (!writtenRow) writtenRow = new logfmt::Snapshot(gate.pendingRow());
            return logfmt::GridGate::WRITTEN;
        }
        return logfmt::GridGate::BUSY;
    });
    grantedAt = 5;  // the lock frees up on the 5th attempt (post + 4 retries)
    logfmt::Snapshot row{}; row.tSec = 300;  // the instant the row is ABOUT
    gate.post(300000u, row);
    for (int i = 0; i < 10; ++i) gate.retry();
    TEST_ASSERT_TRUE_MESSAGE(writtenRow != nullptr, "the pending row was never written");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(300, writtenRow->tSec,
        "a deferred row was written with the retry timestamp, not its own: "
        "the file may be late, it must never be wrong");
    TEST_ASSERT_EQUAL_UINT32(0, gate.lostRows());
    for (int i = 0; i < 5; ++i) gate.retry();  // nothing pending: no phantom work
    TEST_ASSERT_EQUAL_UINT32(0, gate.lostRows());
    delete writtenRow;
}

void test_displaced_pending_row_is_counted_exactly_once() {
    logfmt::GridGate gate;
    gate.setAppender([](uint32_t) { return logfmt::GridGate::BUSY; });  // log frozen
    logfmt::Snapshot a{}; a.tSec = 30;
    logfmt::Snapshot b{}; b.tSec = 60;
    logfmt::Snapshot c{}; c.tSec = 90;
    gate.post(30000u, a);
    gate.retry();
    gate.post(60000u, b);   // a displaced -> LOST (counted)
    gate.retry();
    gate.post(90000u, c);   // b displaced -> LOST (counted)
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(2, gate.lostRows(),
        "lost rows are the HB logskip= number: exactly one count per row that "
        "came due and never reached the file");
    TEST_ASSERT_TRUE(gate.pending());  // c still in play
    // A later unlock writes the newest pending row, and it carries ITS tSec.
    TEST_ASSERT_EQUAL_UINT32(90, gate.pendingRow().tSec);
}

void test_flush_at_close_is_not_a_lost_row() {
    // The dive closer (command task) may write the pending row itself inside
    // the close lock. Claimed as written — and it must NOT surface later as a
    // skip on the next dive.
    logfmt::GridGate gate;
    gate.setAppender([](uint32_t) { return logfmt::GridGate::BUSY; });
    logfmt::Snapshot a{}; a.tSec = 30;
    gate.post(30000u, a);
    TEST_ASSERT_TRUE(gate.pending());
    // (closer appends gate.pendingRow() itself, then:)
    gate.claimPendingWritten();
    TEST_ASSERT_EQUAL_UINT32(0, gate.lostRows());
    logfmt::Snapshot b{}; b.tSec = 60;
    gate.post(60000u, b);
    TEST_ASSERT_EQUAL_UINT32(0, gate.lostRows());  // no retroactive loss of a
                                                   // row that is IN the file
}

// ---------------------------------------------------------------------------
// (3) Output fuse at the write site: provable WITHOUT cycle().
// ---------------------------------------------------------------------------
void test_fuse_condition_never_references_cycle() {
    // A loop that only ever DRIVES the pin — the exact shape of a control task
    // that stalls mid-HIGH: the injector inside cycle() cannot run, the pin
    // stays HIGH. The fuse condition must be computable from the write
    // history alone. It is: pin HIGH written at t=1000, no LOW ever written,
    // and at 1000+maxOn+1 the policy says cut. No cycle() call appears in
    // this test at all — that IS the test.
    outfuse::SolFuse f;
    uint32_t pin = 0;  // the "pin": last level driven
    const uint32_t maxOn = 900;
    f.drive(0u, false);        // boot pulled the pin LOW: the run baseline
    f.drive(1000u, true);
    pin = 1;
    // A stalled task: no drive() calls, no cycle(); an independent evaluator
    // (the supervisor in firmware) queries the condition as time advances.
    for (uint32_t t = 1000u; t <= 1000u + maxOn; ++t += 100u)
        TEST_ASSERT_FALSE_MESSAGE(f.shouldCut(t, maxOn),
                                  "the fuse fired early — at exactly max-on or "
                                  "before, a normal pulse chain could be cut");
    TEST_ASSERT_EQUAL_UINT32(1, pin);  // the pin is still HIGH...
    TEST_ASSERT_TRUE_MESSAGE(f.shouldCut(1000u + maxOn + 1u, maxOn),
                             "the max-on condition depends on something other "
                             "than the pin's own write history — i.e. on a "
                             "control cycle that a stalled task never runs");
    f.cut(1000u + maxOn + 1u);
    pin = f.level() ? 1u : 0u;
    TEST_ASSERT_EQUAL_UINT32(0, pin);  // cut landed
}

void test_latched_fuse_refuses_reassertion_at_the_write_site() {
    // After a trip, drive(true) from ANY path — a recovered stall, a buggy
    // cycle, the injector itself — is refused: the driven level is LOW. The
    // refusal lives at the write site, not in the logic that asks.
    outfuse::SolFuse f;
    f.drive(0u, true);
    f.cut(1000u);
    for (int i = 0; i < 20; ++i)
        TEST_ASSERT_FALSE_MESSAGE(f.drive(1100u + 100u * (uint32_t)i, true),
                                  "a fused output re-asserted HIGH: the latch "
                                  "is the write-site refusal, and this is the "
                                  "case that pins it");
    TEST_ASSERT_EQUAL_UINT32(1, f.trips());
    TEST_ASSERT_TRUE(f.tripped());
    // Re-arm only explicitly (dive-edge / boot), never implicitly.
    f.clear();
    f.drive(5000u, true);
    TEST_ASSERT_TRUE(f.level());
    // Continuous ON is measured from the LOW->HIGH write that opened the
    // valve: a re-armed fuse starts a fresh run at its first HIGH write, not
    // from some earlier idle time at LOW. Idle LOW is not open-valve time.
    TEST_ASSERT_EQUAL_UINT32(0u, f.continuousOnMs(5000u));
}

void test_continuous_on_spans_pile_up_across_rewrites() {
    // Back-to-back HIGH writes with no LOW between are ONE continuous run:
    // max-on is wall time the valve has been open, not one call's request.
    outfuse::SolFuse f;
    f.drive(0u, true);
    for (uint32_t t = 100u; t <= 800u; t += 100u) f.drive(t, true);
    TEST_ASSERT_EQUAL_UINT32(800u, f.continuousOnMs(800u));
    TEST_ASSERT_FALSE(f.shouldCut(800u, 900u));
    TEST_ASSERT_TRUE(f.shouldCut(901u, 900u));
    // A genuine LOW between pulses starts a NEW run (a normal pulse chain):
    // the run begins at the LOW->HIGH write, so a pulse freshly opened at
    // 1000 has been ON 0 ms at 1000 and 350 ms at 1350 — never the idle time
    // between the pulses, which is closed-valve time.
    f.drive(950u, false);
    f.drive(1000u, true);
    TEST_ASSERT_EQUAL_UINT32(0u, f.continuousOnMs(1000u));
    TEST_ASSERT_EQUAL_UINT32(350u, f.continuousOnMs(1350u));
    TEST_ASSERT_FALSE(f.shouldCut(1900u, 900u));   // new run: 900 ms, not past
    TEST_ASSERT_TRUE(f.shouldCut(1901u, 900u));
}

// ---------------------------------------------------------------------------
// (4) Source scan: no portMAX_DELAY anywhere on ControlTask's call graph.
// ---------------------------------------------------------------------------
std::string readFile(const char* path) {
    FILE* f = std::fopen(path, "rb");
    if (!f) return "";
    std::string out;
    char buf[4096];
    size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

// Extract a top-level function body: from the header line through the first
// line that is exactly "}" (project formatting puts closing braces at col 0).
std::string functionBody(const std::string& src, const std::string& header) {
    const size_t at = src.find(header);
    if (at == std::string::npos) return "";
    const size_t end = src.find("\n}\n", at);
    if (end == std::string::npos) return "";
    return src.substr(at, end - at + 3);
}

// Strip comments for the POSITIVE side of a scan: a call that survives only
// inside a comment is not wired. Mutation testing proved the naive version
// lies — commenting out the supervisor's fuse condition left the literal
// strings in the comment and the scan stayed green (mutation M6). The BAN
// side stays on the raw source: a banned string is banned in comments too.
std::string stripComments(const std::string& s) {
    std::string out;
    enum { CODE, LINE, BLOCK } st = CODE;
    for (size_t i = 0; i < s.size();) {
        const char c = s[i];
        const char n = i + 1 < s.size() ? s[i + 1] : '\0';
        if (st == CODE) {
            if (c == '/' && n == '/') { st = LINE; i += 2; continue; }
            if (c == '/' && n == '*') { st = BLOCK; i += 2; continue; }
            out.push_back(c);
            ++i;
        } else if (st == LINE) {
            if (c == '\n') { st = CODE; out.push_back(c); }
            ++i;
        } else {
            if (c == '*' && n == '/') { st = CODE; i += 2; continue; }
            ++i;
        }
    }
    return out;
}

void test_control_task_body_has_no_unbounded_waits_and_no_legacy_appends() {
    const std::string main = readFile("src/main.cpp");
    TEST_ASSERT_TRUE_MESSAGE(!main.empty(), "cannot read src/main.cpp from the test cwd");
    const std::string body = functionBody(main, "static void controlTask(void*) {");
    TEST_ASSERT_TRUE_MESSAGE(!body.empty(), "controlTask body not found: renamed? the "
                                            "scan's subject must stay findable");
    // The rule: ControlTask must contain no unbounded wait anywhere — not on
    // the log mutex, not on anything.
    TEST_ASSERT_TRUE_MESSAGE(body.find("portMAX_DELAY") == std::string::npos,
        "a portMAX_DELAY appeared inside ControlTask: the 1500 ms stall budget "
        "makes every unbounded wait on this task a mid-dive reboot");
    // And the unbounded legacy log entry points must not be called from the
    // control task at all — they are command-task doors now.
    for (const char* banned : {"comms::diveLogRecording", "comms::diveLogAppendGrid",
                               "comms::surfaceLogAppend("}) {
        TEST_ASSERT_TRUE_MESSAGE(body.find(banned) == std::string::npos,
            (std::string("ControlTask calls ") + banned +
            " — an unbounded (portMAX_DELAY) store path; use the bounded "
            "control-task entry points (session.h) instead").c_str());
    }
    // Positive control: the bounded path IS what is called there — in CODE,
    // not in a comment (stripComments: M6 taught this the hard way).
    const std::string code = stripComments(body);
    TEST_ASSERT_TRUE_MESSAGE(code.find("comms::diveLogPostGrid") != std::string::npos,
                            "the grid row no longer goes through the gate");
    TEST_ASSERT_TRUE_MESSAGE(code.find("comms::diveLogRetryGrid") != std::string::npos,
                            "the deferred-row retry vanished from ControlTask");
    TEST_ASSERT_TRUE_MESSAGE(code.find("vTaskDelay") != std::string::npos,
                            "sanity: the scan read an empty body");
}

void test_control_task_store_entry_points_are_bounded() {
    const std::string w = readFile("src/comms/session_wiring.cpp");
    TEST_ASSERT_TRUE_MESSAGE(!w.empty(), "cannot read src/comms/session_wiring.cpp");
    // Every function ControlTask reaches into the stores: no portMAX_DELAY in
    // any of their bodies. (The command-task entry points keep theirs — that
    // is where waiting is allowed; the scan is about the CONTROL path only.)
    for (const char* fn : {"logfmt::GridGate::Result gridAppendAttempt(",
                           "void diveLogPostGrid(", "void diveLogRetryGrid(",
                           "SurfAppend surfaceLogAppendBounded(",
                           "void surfaceRetryPending(",
                           "bool logLockTry("}) {
        const std::string body = functionBody(w, fn);
        TEST_ASSERT_TRUE_MESSAGE(!body.empty(),
                                 (std::string("control-task store entry point not found: ") + fn +
                                  " — renamed? the scan must keep seeing it").c_str());
        TEST_ASSERT_TRUE_MESSAGE(body.find("portMAX_DELAY") == std::string::npos,
                                 (std::string("a portMAX_DELAY appeared in ") + fn +
                                  ": ControlTask must never wait unbounded on a store").c_str());
    }
}

void test_the_supervisor_holds_the_fuse_half_it_promise() {
    // The fuse's third layer (the one that runs when ControlTask never does)
    // must stay wired where it is: the supervisor evaluates shouldCut and
    // drives the cut. If someone "cleans up" the supervisor and drops it, the
    // honest residual window in main.cpp's comment becomes a lie.
    const std::string main = readFile("src/main.cpp");
    const std::string sup = stripComments(functionBody(main, "static void supervisorTask(void*) {"));
    TEST_ASSERT_TRUE_MESSAGE(!sup.empty(), "supervisorTask not found");
    TEST_ASSERT_TRUE_MESSAGE(sup.find("g_solFuse.shouldCut") != std::string::npos,
        "the supervisor no longer evaluates the solenoid fuse: the stuck-HIGH "
        "cut would again depend on the task that is stuck");
    TEST_ASSERT_TRUE_MESSAGE(sup.find("g_solFuse.cut") != std::string::npos,
        "the supervisor no longer latches the fuse after cutting");
    TEST_ASSERT_TRUE_MESSAGE(sup.find("digitalWrite(PIN_SOLENOID, LOW)") != std::string::npos,
        "the supervisor no longer drives the forced cut: evaluating without "
        "actuating is a log line, not a fuse");
    // And the write site keeps its refusal — in code, not in a comment.
    const std::string ctl = stripComments(functionBody(main, "static void controlTask(void*) {"));
    TEST_ASSERT_TRUE_MESSAGE(ctl.find("g_solFuse.drive") != std::string::npos,
        "the pin write no longer goes through the fuse gate");
}

}  // namespace

void setUp() {}
void tearDown() {}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_worst_case_row_fill_fits_the_control_budget);
    RUN_TEST(test_fill_cost_grows_with_depth_not_worse_than_quadratic);
    RUN_TEST(test_capped_schedule_reports_itself);
    RUN_TEST(test_gate_defers_and_retries_a_busy_append);
    RUN_TEST(test_busy_row_is_kept_written_once_and_never_double_counted);
    RUN_TEST(test_displaced_pending_row_is_counted_exactly_once);
    RUN_TEST(test_flush_at_close_is_not_a_lost_row);
    RUN_TEST(test_fuse_condition_never_references_cycle);
    RUN_TEST(test_latched_fuse_refuses_reassertion_at_the_write_site);
    RUN_TEST(test_continuous_on_spans_pile_up_across_rewrites);
    RUN_TEST(test_control_task_body_has_no_unbounded_waits_and_no_legacy_appends);
    RUN_TEST(test_control_task_store_entry_points_are_bounded);
    RUN_TEST(test_the_supervisor_holds_the_fuse_half_it_promise);
    return UNITY_END();
}
