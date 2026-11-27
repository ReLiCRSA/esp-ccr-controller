// ============================================================================
// test_configbox — the applied configuration as a published generation.
//
// ctrl::Config used to be one global that the settings path wrote field by
// field while SensorTask, SupervisorTask and ControlTask read it. On a 32-bit
// core no word is torn; what was torn was the PAIR — a new ALLO_HIGH beside the
// old ALLO_CUTOFF, a lowered GF_LOW beside the old GF_HIGH — and settings_apply
// .cpp exists precisely to make those combinations impossible, so the global
// handed readers a state the applier guarantees cannot exist. Everything here
// is about that: a reader gets one whole generation, and the publishing order
// the firmware depends on is pinned in the source, not asserted in prose.
//
// The concurrency case is the evidence the mutex is doing its job: every
// generation is a deterministic function of one seed, so a read that blended
// two publishes cannot decode to a single seed.
// ============================================================================
#include <unity.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "control/configbox.h"

void setUp() {}
void tearDown() {}

namespace {

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

std::string stripComments(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool inLine = false, inBlock = false;
    for (size_t i = 0; i < src.size(); ++i) {
        const char c = src[i];
        const char nx = i + 1 < src.size() ? src[i + 1] : '\0';
        if (inLine) {
            if (c == '\n') { inLine = false; out += c; }
            continue;
        }
        if (inBlock) {
            if (c == '*' && nx == '/') { inBlock = false; ++i; }
            continue;
        }
        if (c == '/' && nx == '/') { inLine = true; ++i; continue; }
        if (c == '/' && nx == '*') { inBlock = true; ++i; continue; }
        out += c;
    }
    return out;
}

/// A generation derived wholly from `seed`, so every field names the same seed
/// or the read is a blend. The relations are the ones settings_apply enforces
/// (ascending ladder, alarm edge below the injection wall, pulse under the
/// max-on fuse), so a blend is both a decode failure and a rule violation.
ctrl::Config fromSeed(int seed) {
    ctrl::Config c{};
    c.deco.gfLow = static_cast<float>(seed);
    c.deco.gfHigh = static_cast<float>(seed) + 5.0f;
    c.sp.sp[0] = static_cast<float>(seed) * 0.01f;
    c.sp.sp[1] = c.sp.sp[0] + 0.1f;
    c.sp.sp[2] = c.sp.sp[1] + 0.1f;
    c.sp.sp[3] = c.sp.sp[2] + 0.1f;
    c.alarms.highBar = static_cast<float>(seed) * 0.001f;
    c.inj.highWallBar = c.alarms.highBar + 0.1f;
    c.inj.maxOnMs = static_cast<uint16_t>(200 + seed);
    c.inj.pulseMs = static_cast<uint16_t>(100 + seed);
    return c;
}

/// The seed this generation was built from, or -1 when the fields do not agree
/// on one — which is exactly a blended read.
int seedOf(const ctrl::Config& c) {
    const int seed = static_cast<int>(c.deco.gfLow);
    if (c.deco.gfHigh != static_cast<float>(seed) + 5.0f) return -1;
    if (c.sp.sp[0] != static_cast<float>(seed) * 0.01f) return -1;
    if (c.alarms.highBar != static_cast<float>(seed) * 0.001f) return -1;
    if (c.inj.highWallBar != c.alarms.highBar + 0.1f) return -1;
    if (c.inj.maxOnMs != static_cast<uint16_t>(200 + seed)) return -1;
    if (c.inj.pulseMs != static_cast<uint16_t>(100 + seed)) return -1;
    return seed;
}

}  // namespace

// MUST RUN FIRST: "nothing published yet" is the state before setup() has run,
// and the box is process-global — one boot per binary, as on the device.
void test_nothing_published_leaves_the_callers_value_alone() {
    TEST_ASSERT_EQUAL_UINT32(0, ctrl::configGeneration());
    ctrl::Config c{};
    c.sp.sp[0] = 0.40f;      // a caller's seeded default (defaults.h)
    c.deco.waterDensity = 1.025f;
    TEST_ASSERT_FALSE_MESSAGE(ctrl::configRead(c),
                              "a read before the first publish must say so, not hand back zeros");
    // Untouched, both fields: a value-initialised struct is presentMask = 0 and
    // maxDutyPct = 0, which reads as "no cells fitted, no oxygen ever".
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.40f, c.sp.sp[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.025f, c.deco.waterDensity);
}

void test_a_publish_replaces_the_whole_generation() {
    ctrl::configPublish(fromSeed(31));
    TEST_ASSERT_EQUAL_UINT32(1, ctrl::configGeneration());
    ctrl::Config c{};
    TEST_ASSERT_TRUE(ctrl::configRead(c));
    TEST_ASSERT_EQUAL_INT(31, seedOf(c));

    // A second publish must not merge with the first field by field — that is
    // the old global's behaviour, and it is what produced the impossible pairs.
    ctrl::configPublish(fromSeed(32));
    TEST_ASSERT_EQUAL_UINT32(2, ctrl::configGeneration());
    TEST_ASSERT_TRUE(ctrl::configRead(c));
    TEST_ASSERT_EQUAL_INT(32, seedOf(c));
    TEST_ASSERT_EQUAL_UINT16_MESSAGE(232, c.inj.maxOnMs,
                                     "partial update: maxOnMs still holds the previous generation");
}

void test_reads_under_a_publishing_thread_never_blend() {
    // Writer = the settings/command task, reader = the control chain's shape:
    // one whole read per iteration, checking the fields agree on one seed.
    std::atomic<bool> done{false};
    // Seeded above what the cases before this one published (the box is
    // process-global, one boot per binary): the monotonicity check below is
    // about the reader never going backwards, so it needs a range the writer
    // actually moves forward through from here.
    const int kFirst = 100;
    const int kGenerations = 4000;
    std::thread writer([&] {
        for (int i = kFirst; i < kFirst + kGenerations; ++i) ctrl::configPublish(fromSeed(i));
        done = true;
    });
    int reads = 0, generations = 0;
    int lastSeed = 0;
    while (!done) {
        ctrl::Config c{};
        if (!ctrl::configRead(c)) continue;
        ++reads;
        const int seed = seedOf(c);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(-1, seed,
                                      "blended generation: fields from two publishes");
        // A generation never goes backwards: the reader sees a monotone
        // sequence, so "the newest rules" is a fact and not a hope.
        TEST_ASSERT_TRUE_MESSAGE(seed >= lastSeed, "a read returned an older generation");
        if (seed != lastSeed) ++generations;
        lastSeed = seed;
    }
    writer.join();
    TEST_ASSERT_TRUE_MESSAGE(reads > 0, "the reader never got a generation: the box is dead");
    printf("  configbox: %d reads, %d generations seen, none blended\n", reads, generations);
}

void test_the_settings_path_applies_onto_a_copy_and_publishes_once() {
    // hookSettings is the only mid-flight writer, and the order is the whole
    // contract: read the current generation (so the fields it does not write —
    // injection wall, max-on, duty window, proof pulses — carry forward), apply
    // onto that copy, publish the finished result, then queue it for the chain.
    // Publish BEFORE postConfig: ControlTask reads the generation at the top of
    // its cycle and takes the queued one in step 0, so a tightened limiter
    // reaches the pin fuse no later than it reaches the engine.
    const std::string sw = stripComments(readFile("src/comms/session_wiring.cpp"));
    const size_t hook = sw.find("void hookSettings(");
    TEST_ASSERT_TRUE_MESSAGE(hook != std::string::npos, "hookSettings gone?");
    const std::string body = sw.substr(hook, sw.find("\n}", hook) - hook);

    const size_t read = body.find("ctrl::configRead(");
    const size_t apply = body.find("applySettings(");
    const size_t publish = body.find("ctrl::configPublish(");
    const size_t post = body.find("postConfig(");
    TEST_ASSERT_TRUE_MESSAGE(read != std::string::npos && apply != std::string::npos &&
                             publish != std::string::npos && post != std::string::npos,
                             "hookSettings no longer reads, applies, publishes and queues");
    TEST_ASSERT_TRUE_MESSAGE(read < apply,
                             "the apply must start from the published generation: which fields "
                             "does it then carry from boot?");
    TEST_ASSERT_TRUE_MESSAGE(apply < publish,
                             "the generation must be published after it is finished, not before");
    TEST_ASSERT_TRUE_MESSAGE(publish < post,
                             "publish must precede postConfig: the solenoid fuse reads the "
                             "published generation and must not lag the chain it limits");
    // The queued value must be the same finished struct that was published, not
    // a re-read — a re-read could pick up someone else's generation.
    TEST_ASSERT_TRUE_MESSAGE(body.find("postConfig(cfg)") != std::string::npos,
                             "postConfig no longer queues the published struct");
}

void test_no_task_reads_the_old_global() {
    // The global is what made an impossible pair readable. If a task is back on
    // it, the coherence tests above keep passing while the firmware regresses,
    // so the name is banned. Comments are exempt from the broad ban — prose
    // needs to be able to say what the old thing was — but the two forms only
    // code can take, a declaration and a field read, are banned raw, with the
    // comments in. That is what survives someone re-adding the extern, and what
    // survives a commented-out fuse (test_controlplane's mutation M6).
    const char* files[] = {"src/main.cpp", "src/comms/session_wiring.cpp",
                           "src/comms/telemetry_bridge.cpp", "src/hud.cpp"};
    for (const char* f : files) {
        const std::string raw = readFile(f);
        TEST_ASSERT_TRUE_MESSAGE(!raw.empty(), f);
        TEST_ASSERT_NULL_MESSAGE(std::strstr(raw.c_str(), "g_ctrlCfg."),
                                 (std::string(f) + " reads a field off g_ctrlCfg again").c_str());
        TEST_ASSERT_NULL_MESSAGE(std::strstr(raw.c_str(), "extern ctrl::Config g_ctrlCfg"),
                                 (std::string(f) + " re-declares the old global").c_str());
        TEST_ASSERT_NULL_MESSAGE(std::strstr(stripComments(raw).c_str(), "g_ctrlCfg"),
                                 (std::string(f) + " uses the old global outside a comment: the applied "
                                                   "rules are a published generation now "
                                                   "(control/configbox.h)").c_str());
    }
    // And each consumer takes a generation per iteration, not one at entry:
    // a SET from the handset reaches the sensor's ruler, the pin fuse and the
    // dive row on the next tick, which is the latency the command path promises.
    // A single read outside the loop would pass a "does it ever read" scan and
    // leave the task running on its boot value for the rest of the dive.
    const std::string mainCpp = stripComments(readFile("src/main.cpp"));
    for (const char* site : {"sensorTask", "controlTask", "supervisorTask"}) {
        const size_t at = mainCpp.find("static void " + std::string(site));
        TEST_ASSERT_TRUE_MESSAGE(at != std::string::npos, site);
        const size_t end = mainCpp.find("\nstatic ", at + 1);
        const std::string body = mainCpp.substr(at, end - at);
        const size_t loop = body.find("for (;;)");
        TEST_ASSERT_TRUE_MESSAGE(loop != std::string::npos, site);
        TEST_ASSERT_TRUE_MESSAGE(body.find("ctrl::configRead(", loop) != std::string::npos,
                                 (std::string(site) + " never re-reads a generation inside its loop: "
                                                   "it runs on whatever boot handed it").c_str());
    }
}

void test_boot_publishes_before_any_reader_task_exists() {
    // setup() is the other writer, and the ordering is what makes every
    // "unreachable" fallback in the tasks genuinely unreachable: a task created
    // before the publish would start on its seed value instead of the applied
    // rules.
    const std::string mainCpp = stripComments(readFile("src/main.cpp"));
    const size_t setup = mainCpp.find("void setup()");
    TEST_ASSERT_TRUE_MESSAGE(setup != std::string::npos, "setup() gone?");
    const std::string body = mainCpp.substr(setup);
    const size_t build = body.find("buildControlConfig(cfg)");
    const size_t apply = body.find("applySettings(g_settings, cfg)");
    const size_t publish = body.find("ctrl::configPublish(cfg)");
    const size_t loop = body.find("new (std::nothrow) ctrl::Loop(cfg)");
    const size_t task = body.find("xTaskCreatePinnedToCore(sensorTask");
    TEST_ASSERT_TRUE_MESSAGE(build != std::string::npos && apply != std::string::npos &&
                             publish != std::string::npos && loop != std::string::npos &&
                             task != std::string::npos,
                             "setup() no longer builds, applies and publishes one boot config");
    TEST_ASSERT_TRUE_MESSAGE(build < apply, "boot must apply onto the defaults");
    TEST_ASSERT_TRUE_MESSAGE(apply < publish,
                             "the boot generation must be published after the NVS clamps ran");
    TEST_ASSERT_TRUE_MESSAGE(publish < loop,
                             "the chain must be built from the published config, not the global");
    TEST_ASSERT_TRUE_MESSAGE(publish < task,
                             "publishing after the tasks exist lets a task run on its seed value");
}

void test_a_reader_can_say_which_generation_it_holds() {
    // The box fixes coherence; this fixes the CLAIM about coherence. A reader that
    // wants to report "the rules this cycle ran under" cannot ask
    // configGeneration() afterwards — that answers whatever the settings path
    // published in the meantime, which is the same half-pair problem the box was
    // built to remove. The number has to come out of the same read as the values.
    ctrl::configPublish(fromSeed(77));
    ctrl::Config c{};
    uint32_t gen = 0;
    TEST_ASSERT_TRUE(ctrl::configRead(c, &gen));
    const uint32_t readGen = gen;
    TEST_ASSERT_EQUAL_INT(77, seedOf(c));

    // Publish again: the number the earlier read reported must still be the number
    // that describes THOSE values.
    ctrl::configPublish(fromSeed(78));
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(readGen, gen,
                                     "the generation a read reported moved under it: the pair is not "
                                     "a pair");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(readGen + 1, ctrl::configGeneration(),
                                     "configGeneration() no longer counts publishes");

    // The "nothing published yet" half of the overload — leaving BOTH the caller's
    // struct and its generation variable alone — is graded by the first case in
    // this file, which is the only place that state exists: the box is
    // process-global, one boot per binary, and by here it has been published.
}

void test_only_the_boot_and_the_settings_path_publish() {
    // configbox.h says the settings path is the only mid-flight writer, and the
    // file's whole argument depends on that: two mid-flight writers means last
    // writer wins, and the pairs settings_apply orders stop meaning anything. A
    // comment can be ignored; a count in the source cannot. Every call site of
    // configPublish in src/ is enumerated here, and a new one fails this test with
    // a message that says what the new writer must do instead — read the current
    // generation, apply onto that copy, publish the result — rather than letting a
    // second writer in through an exception nobody reviews.
    const char* files[] = {"src/main.cpp", "src/comms/session_wiring.cpp",
                           "src/comms/telemetry_bridge.cpp", "src/comms/cmd_router.cpp",
                           "src/comms/web_server.cpp", "src/control/configbox.cpp"};
    int sites = 0;
    for (const char* f : files) {
        const std::string src = stripComments(readFile(f));
        if (f == std::string("src/control/configbox.cpp")) continue;   // the definition
        TEST_ASSERT_TRUE_MESSAGE(!src.empty(), f);
        for (size_t at = src.find("configPublish("); at != std::string::npos;
             at = src.find("configPublish(", at + 1)) {
            // The declaration in configbox.h is not a call; nothing here includes
            // a header that declares it, so every hit in these files is a call.
            ++sites;
            TEST_ASSERT_TRUE_MESSAGE(f == std::string("src/main.cpp") ||
                                     f == std::string("src/comms/session_wiring.cpp"),
                                     (std::string(f) +
                                      " publishes a configuration generation. The only mid-flight "
                                      "writer is the settings path: read the published generation, "
                                      "apply onto that copy, then publish — and add this file here "
                                      "only if that shape is what it does.").c_str());
        }
    }
    // Two, not one and not three: setup()'s boot publish and hookSettings'.
    TEST_ASSERT_EQUAL_INT_MESSAGE(2, sites,
                                  "the number of configPublish call sites moved: the single-writer "
                                  "rule is the box's contract");

    // And the boot publisher must be a publisher of an APPLIED config, not of a
    // struct nobody clamped — graded on setup()'s own text (see the boot test).
}

void test_the_heartbeat_states_the_generation_pair() {
    // The claim "the chain is running the rules the last SET produced" is now a
    // bench-visible number, so the bench has to be able to lose it. HB carries
    // `cfggen=<published>/<chain>`; the chain half must come from the read that
    // carried the values, never from configGeneration(), which would report what
    // the settings path published by now rather than what the chain is running.
    const std::string mainCpp = stripComments(readFile("src/main.cpp"));
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(mainCpp.c_str(), "cfggen=%u/%u"),
                                 "HB no longer reports the config generation pair");
    const size_t at = mainCpp.find("cfggen=%u/%u");
    // The format string and its argument list are one printf statement: the
    // arguments sit below the format, so the window has to cover them.
    const std::string tail = mainCpp.substr(at, 1400);
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(tail.c_str(), "ctrl::configGeneration()"),
                                 "HB's published half is not the box's count");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(tail.c_str(), "g_chainCfgGen"),
                                 "HB's chain half is not the generation ControlTask read");

    const size_t ctrl = mainCpp.find("static void controlTask");
    TEST_ASSERT_TRUE(ctrl != std::string::npos);
    const std::string body = mainCpp.substr(ctrl, mainCpp.find("\nstatic ", ctrl + 1) - ctrl);
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(body.c_str(), "configRead(g_controlRules, &cfgGen)"),
                                 "controlTask no longer takes the generation WITH the values");
    TEST_ASSERT_NOT_NULL_MESSAGE(std::strstr(body.c_str(), "g_chainCfgGen = cfgGen"),
                                 "controlTask no longer records which generation it applied");
    // Appended, never reordered: soak_logger.py and hw_accept.py anchor on the
    // leading HB fields, so the new field must sit after contskip=.
    const size_t hb = mainCpp.find("HB st=");
    const std::string fmt = mainCpp.substr(hb, mainCpp.find("\\n", hb) - hb);
    const size_t skip = fmt.find("contskip="), cfg = fmt.find("cfggen=");
    TEST_ASSERT_TRUE_MESSAGE(cfg != std::string::npos && skip != std::string::npos && cfg > skip,
                             "cfggen= is not appended after contskip=: HB fields are append-only");
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_nothing_published_leaves_the_callers_value_alone);
    RUN_TEST(test_a_publish_replaces_the_whole_generation);
    RUN_TEST(test_reads_under_a_publishing_thread_never_blend);
    RUN_TEST(test_the_settings_path_applies_onto_a_copy_and_publishes_once);
    RUN_TEST(test_no_task_reads_the_old_global);
    RUN_TEST(test_boot_publishes_before_any_reader_task_exists);
    RUN_TEST(test_a_reader_can_say_which_generation_it_holds);
    RUN_TEST(test_only_the_boot_and_the_settings_path_publish);
    RUN_TEST(test_the_heartbeat_states_the_generation_pair);
    return UNITY_END();
}
