// ============================================================================
// cmd_router.cpp — see cmd_router.h.
//
// The ordering inside exec() is a safety property: parse fully before any
// mutation, and check the dive-lock before the registry check, so a command
// that is refused cannot half-apply. Nothing here touches hardware; the
// transport acts on the returned action bits.
// ============================================================================
#include "cmd_router.h"

#include "util/numfmt.h"

#include <cstdio>
#include <cstring>

namespace comms {

using proto::CalAction;
using proto::Command;
using proto::Kind;

// ---- reply helpers ----------------------------------------------------------

void CommandRouter::replyOk(Response& r, const char* note) {
    if (note && *note)
        r.len = static_cast<size_t>(std::snprintf(r.text, sizeof(r.text), "OK %s\n", note));
    else
        r.len = proto::formatOk(r.text, sizeof(r.text));
    terminate(r);
}

void CommandRouter::replyErr(Response& r, const char* why) {
    r.len = proto::formatErr(why, r.text, sizeof(r.text));
    terminate(r);
}

void CommandRouter::replyText(Response& r, const char* text, size_t n) {
    if (n == 0 || n >= sizeof(r.text)) {
        replyErr(r, "reply overflow");
        return;
    }
    std::memcpy(r.text, text, n);
    r.len = n;
    terminate(r);
}

// The one rule every reply path must satisfy: r.text is a NUL-terminated C
// string whose length is exactly r.len, no matter what the buffer held before.
// The transports hand r.text to a sink that treats it as a string, so a reply
// that runs into leftover bytes from the previous command puts those bytes on
// the wire — the failure test_router now guards against.
void CommandRouter::terminate(Response& r) {
    const size_t cap = sizeof(r.text);
    if (r.len >= cap) r.len = cap - 1;      // a formatter overran: keep it short
    r.text[r.len] = '\0';
}

void CommandRouter::replyInPlace(Response& r, size_t n) {
    if (n == 0 || n >= sizeof(r.text)) {   // formatter refused, or no room:
        replyErr(r, "reply overflow");     // never emit half a line
        return;
    }
    r.len = n;
    terminate(r);
}

// ---- the settings schema text, rendered once at boot ------------------------
//
// The K, lines describe the settings table, which is fixed at compile time — so
// every GET SCHEMA reply is the same bytes, and formatting it per command buys
// nothing. It also cost a 1024-byte stack temporary and three %.6g conversions
// per key (newlib float printf, the thing that panicked the uart2 task) on the
// task that answers a diver. It is now rendered once by primeSchema() from
// setup(), before any small-stack task exists, and every later GET SCHEMA is a
// copy.
//
// The truncation rule is deliberate and loud: a partial list served as if it
// were complete is worse than a short answer, because a handset that cannot see
// a key cannot set it and has no way to know why. primeSchema() returns the
// length, the caller compares it against MAX_SCHEMA, and test_schema proves it
// fits with room to spare.

namespace {

char g_schema[CommandRouter::MAX_SCHEMA];
size_t g_schemaLen = 0;
bool g_schemaReady = false;

size_t renderSchema(char* out, size_t cap) {
    size_t cnt = 0;
    const settings::Def* defs = settings::Registry::definitions(cnt);
    size_t p = 0;
    for (size_t i = 0; i < cnt; ++i) {
        char line[96];
        char tc = '?';
        switch (defs[i].type) {
            case settings::Type::FLOAT: tc = 'F'; break;
            case settings::Type::INT16: tc = 'I'; break;
            case settings::Type::UINT8: tc = 'U'; break;
            case settings::Type::BOOL:  tc = 'B'; break;
            case settings::Type::STRING: tc = 'S'; break;
        }
        int n;
        if (defs[i].type == settings::Type::STRING)
            n = std::snprintf(line, sizeof(line), "K,%s,%c,%u,%u,%u,%s\n",
                              defs[i].key, tc, static_cast<unsigned>(defs[i].minv),
                              (unsigned)defs[i].maxv, defs[i].flags,
                              defs[i].defStr ? defs[i].defStr : "");
        else
            n = std::snprintf(line, sizeof(line),
                              "K,%s,%c,%.6g,%.6g,%u,%.6g\n",  /*float-ok: compile-time settings table, rendered once by Session::begin before the small-stack tasks exist*/
                              defs[i].key, tc, static_cast<double>(defs[i].minv),
                              static_cast<double>(defs[i].maxv), defs[i].flags,
                              static_cast<double>(defs[i].def));
        if (n <= 0) continue;                                   // unprintable key
        if (p + static_cast<size_t>(n) >= cap) return p;        // does not fit
        std::memcpy(out + p, line, static_cast<size_t>(n));
        p += static_cast<size_t>(n);
    }
    return p;
}

} // namespace

size_t primeSchema() {
    g_schemaLen = renderSchema(g_schema, sizeof(g_schema));
    g_schemaReady = g_schemaLen > 0 && g_schemaLen < sizeof(g_schema);
    return g_schemaLen;
}

size_t schemaLen() { return g_schemaLen; }

// ---- shared formatters -------------------------------------------------------

proto::DecoInfo CommandRouter::decoInfoOf(const ctrl::Loop& loop, bool hasVote) {
    proto::DecoInfo d{};
    const deco::Zhl16Ccr& z = loop.deco();
    const double amb = z.depthToBar(loop.depthSmoothedM());
    const double ceilingM = z.ceilingM();
    d.ceilingM = ceilingM > 0.0f ? static_cast<float>(ceilingM) : 0.0f;
    if (hasVote) {
        double ndl = z.ndlMinutes(amb, loop.votedBar());
        d.ndlMin = ndl >= 480.0 ? 480 : (ndl < 0.0 ? 0 : static_cast<int>(ndl));
    } else {
        d.ndlMin = -1;  // unknown: no vote, no honest NDL to report
    }
    const deco::Zhl16Ccr::Schedule sch = z.schedule(amb);
    d.decoTotalMin = static_cast<float>(sch.totalTimeMin);
    d.stopCount = sch.count > proto::MAX_STOPS ? proto::MAX_STOPS : sch.count;
    for (int i = 0; i < d.stopCount; ++i) {
        d.stops[i].depthM = sch.stops[i].depthM;
        d.stops[i].minutes = sch.stops[i].minutes;
    }
    d.cnsPct = static_cast<float>(z.cnsPct());
    return d;
}

// The protocol flags byte is 8 bits; alarm bits 6..9 are reported as SENSOR
// trouble rather than dropped silently (they are all sensor/fuel failures).
static uint8_t telemetryFlags(const ctrl::Loop& loop) {
    const uint16_t a = loop.alarmBits();
    uint8_t f = 0;
    if (a & (1u << alarms::BIT_LOW_PPO2))    f |= 1 << 0;
    if (a & (1u << alarms::BIT_HIGH_PPO2))   f |= 1 << 1;
    if (a & (1u << alarms::BIT_CELL_DRIFT))  f |= 1 << 2;
    if (a & ((1u << alarms::BIT_SENSOR_FAIL) | (1u << alarms::BIT_SENSOR_STALE) |
             (1u << alarms::BIT_STUCK_OPEN) | (1u << alarms::BIT_HYPOXIA)))
        f |= 1 << 3;
    if (a & (1u << alarms::BIT_BATTERY))     f |= 1 << 4;
    if (a & (1u << alarms::BIT_SP_CLAMP))    f |= 1 << 5;
    if (loop.cellproof().state == cellproof::State::FAIL) f |= 1 << 6;
    return f;
}

// ---- wifi / dive lifecycle ---------------------------------------------------

void CommandRouter::applyWifi(Response& r, bool on) {
    if (loop_.diveActive() && on) {
        // The radio is off *because* we are diving; re-opening it mid-dive is
        // not a convenience, it is 40-70 mA and an antenna on the loop.
        replyErr(r, "wifi locked in dive");
        return;
    }
    if (wifiOn_ == on) { replyOk(r); return; }
    wifiOn_ = on;
    if (!on && loop_.diveActive()) wifiAutoOff_ = false;  // diver's own choice
    r.actions |= CommandRouter::ACT_WIFI;
    replyOk(r);
}

void CommandRouter::startDive(Response& r) {
    if (loop_.diveActive()) { replyErr(r, "dive active"); return; }
    // The dive gate flips here (dive-locked refusals and telemetry mode are
    // true on the very next read); the chain's dive-state work lands at the
    // next control tick — see the post* contract in loop.h.
    loop_.postStartDive(nowMs_);
    logExportIdx_ = 0;
    if (wifiOn_) {            // drop the radio, remember to restore it later
        wifiOn_ = false;
        wifiAutoOff_ = true;
        r.actions |= ACT_WIFI;
    } else {
        wifiAutoOff_ = false;  // diver had it off: dive end must not turn it on
    }
    r.actions |= ACT_DIVE_START;
    replyOk(r, "DIVE");
}

void CommandRouter::endDive(Response& r) {
    if (!loop_.diveActive()) { replyErr(r, "no dive"); return; }
    loop_.postEndDive(nowMs_);
    r.actions |= ACT_DIVE_END;
    if (wifiAutoOff_) {        // restore what DIVE START took away
        wifiAutoOff_ = false;
        wifiOn_ = true;
        r.actions |= ACT_WIFI;
    }
    replyOk(r, "DIVE");
}

// ---- GET ---------------------------------------------------------------------

void CommandRouter::buildCellsLine(Response& r) {
    char buf[96];
    const int n = std::snprintf(buf, sizeof(buf), "CELLS,%d,%03o,%03o,%d\n",
                                static_cast<int>(loop_.pulseTestState()),
                                loop_.cellTestPassMask(), loop_.cellTestFailMask(),
                                static_cast<int>(loop_.cellTestRequested()));
    replyText(r, buf, static_cast<size_t>(n));
}

// `CALAIR,<code>,<age_s>,<samples>,<spread>,<raw0>,<raw1>,<raw2>,<warmup_left_s>`
// The air calibration's real verdict, on the same wire the request arrived on.
// Raw ADC counts (not ppO2) and the worst per-cell spread: those are the numbers
// an installer needs to tell a dead cell from a fouled one, and until now they
// existed only on the USB console. `<code>` is bridge::CalAirCode, the same
// vocabulary as the `E,` id 7 event value. No calibration since this power-up
// answers code 0 with zeros — a stated nothing, not an error a client has to
// special-case and not a result it could mistake for one.
void CommandRouter::buildCalAirLine(Response& r) {
    char buf[128], r0[16], r1[16], r2[16], ws[16];
    const uint8_t code = calAir_ ? static_cast<uint8_t>(calAir_->code) : bridge::CALAIR_NONE;
    const uint32_t at = calAir_ ? static_cast<uint32_t>(calAir_->atMs) : 0u;
    const uint32_t age = (calAir_ && code) ? (nowMs_ - at) / 1000u : 0u;
    const int n = std::snprintf(buf, sizeof(buf), "CALAIR,%u,%u,%u,%s,%s,%s,%s,%u\n",
                                static_cast<unsigned>(code),
                                static_cast<unsigned>(age),
                                static_cast<unsigned>(calAir_ ? calAir_->samples : 0u),
                                numfmt::fixed(ws, sizeof(ws),
                                              calAir_ ? calAir_->spreadWorst : 0.0f, 0),
                                numfmt::fixed(r0, sizeof(r0),
                                              calAir_ ? calAir_->rawAvg[0] : 0.0f, 0),
                                numfmt::fixed(r1, sizeof(r1),
                                              calAir_ ? calAir_->rawAvg[1] : 0.0f, 0),
                                numfmt::fixed(r2, sizeof(r2),
                                              calAir_ ? calAir_->rawAvg[2] : 0.0f, 0),
                                static_cast<unsigned>(calAir_ ? calAir_->warmupLeftS : 0u));
    replyText(r, buf, static_cast<size_t>(n));
}

void CommandRouter::getSpecial(Response& r, const char* key) {
    if (std::strcmp(key, "T") == 0) {
        proto::Telemetry t;
        t.depthM = loop_.depthSmoothedM();
        t.votedPpo2 = loop_.votedBar();
        for (int i = 0; i < 3; i++) t.cells[i] = loop_.cellBar(i);
        t.setpoint = loop_.setpointBar();
        // SAFE is a fact about the machine, and the router is the one that
        // knows it (safeLockout_): the T, line must say SAFE rather than the
        // STANDBY that a control chain which is not running would report.
        t.mode = safeLockout_ ? 3 : (loop_.diveActive() ? (loop_.autoMode() ? 1 : 2) : 0);
        t.flags = telemetryFlags(loop_);
        t.battV = loop_.vbatVolts();
        replyInPlace(r, proto::formatTelemetry(t, r.text, sizeof(r.text)));
        return;
    }
    if (std::strcmp(key, "D") == 0) {
        // formatted into r.text: a 1024-byte temporary here sat on the uart2
        // task's stack, on top of the 1024-byte Response copies around it
        replyInPlace(r, proto::formatDeco(decoInfoOf(loop_, loop_.haveVote()),
                                          r.text, sizeof(r.text)));
        return;
    }
    if (std::strcmp(key, "TIME") == 0) {
        // TIME,<epoch>,<source>,<age_s>. Epoch 0 means there is no clock, and
        // that is the answer a bench needs to hear, not a 1970 date.
        // source: 0 unset, 1 set since this boot, 2 carried across a reset.
        char buf[48];
        const uint32_t e = clock_ ? wallclock::epoch(*clock_, nowMs_) : 0u;
        const uint32_t src = clock_ ? clock_->source : (uint32_t)wallclock::UNSET;
        const uint32_t age = clock_ ? wallclock::ageS(*clock_, nowMs_) : 0u;
        const int n = std::snprintf(buf, sizeof(buf), "TIME,%u,%u,%u\n",
                                    e, src, age);
        replyText(r, buf, static_cast<size_t>(n));
        return;
    }
    if (std::strcmp(key, "CELLS") == 0) { buildCellsLine(r); return; }
    if (std::strcmp(key, "PROOF") == 0) {
        const cellproof::Output& cp = loop_.cellproof();
        char buf[64];
        const int n = std::snprintf(buf, sizeof(buf), "PROOF,%d,%03o,%03o,%u\n",
                                    static_cast<int>(cp.state), cp.unprovenHighMask,
                                    loop_.proofExclusion(), static_cast<unsigned>(cp.attempts));
        replyText(r, buf, static_cast<size_t>(n));
        return;
    }
    // The air calibration's verdict, and progress while it samples (12 s: a
    // diver who started it needs to know whether it is running, refused or
    // done — the `OK CAL AIR` reply only ever meant "requested").
    if (std::strcmp(key, "CALAIR") == 0) { buildCalAirLine(r); return; }
    if (std::strcmp(key, "PRE") == 0) {
        char buf[64];
        int p = std::snprintf(buf, sizeof(buf), "PRE,");
        for (int s = 0; s < predive::S_COUNT; ++s)
            p += std::snprintf(buf + p, sizeof(buf) - p, "%d,",
                               static_cast<int>(loop_.prediveStatus(static_cast<predive::Step>(s))));
        p += std::snprintf(buf + p, sizeof(buf) - p, "%d\n", loop_.predivePassed() ? 1 : 0);
        replyText(r, buf, static_cast<size_t>(p));
        return;
    }
    if (std::strcmp(key, "FAULT") == 0) {
        char buf[64], duty[16];
        const int n = std::snprintf(buf, sizeof(buf), "FAULT,%d,%d,%s\n",
                                    loop_.injector().stuckOpenLatched ? 1 : 0,
                                    loop_.outputFuseTripped() ? 1 : 0,
                                    numfmt::fixed(duty, sizeof(duty),
                                                  loop_.solDutyPct(nowMs_), 1));
        replyText(r, buf, static_cast<size_t>(n));
        return;
    }
    if (std::strcmp(key, "HIST") == 0) {
        if (!hist_) { replyErr(r, "no histogram"); return; }
        replyInPlace(r, cellhist::formatLines(*hist_, nowMs_, r.text, sizeof(r.text)));
        return;
    }
    // The web Settings page needs each key's type, range and flags to put a
    // min/max on its inputs and grey out what a dive locks; the handset gets
    // the same view. Additive display key — exec() asks the registry before
    // it asks here, so it can never shadow a real setting.
    if (std::strcmp(key, "SCHEMA") == 0) {
        // answered from the boot-time rendering above
        if (!g_schemaReady) { replyErr(r, "schema not primed"); return; }
        if (g_schemaLen >= sizeof(r.text)) { replyErr(r, "reply overflow"); return; }
        std::memcpy(r.text, g_schema, g_schemaLen);
        r.len = g_schemaLen;
        terminate(r);
        return;
    }

    replyErr(r, "unknown key");
}

// ---- dispatch ----------------------------------------------------------------

void CommandRouter::exec(const char* line, Response& r) {
    r.len = 0;
    r.actions = ACT_NONE;
    r.text[0] = '\0';
    Command cmd;
    const char* err = nullptr;
    if (!proto::parseCommand(line, cmd, &err)) {
        replyErr(r, err ? err : "bad command");
        return;
    }

    // SAFE mode lockout: checked after parsing (so a malformed line still
    // reports its real syntax problem) and before anything else, because the
    // point is that no command can reach a chain that is not running.
    if (safeLockout_ && cmd.kind != Kind::ACK) {
        replyErr(r, "safe mode");
        return;
    }

    switch (cmd.kind) {
        case Kind::GET:
            if (std::strcmp(cmd.key, "*") == 0) {
                replyInPlace(r, reg_.listAll(r.text, sizeof(r.text)));
            } else {
                // Registry first: real keys (D1, D2, CELLS_..., CAL_...) must
                // never be shadowed by a display key. Only keys the registry
                // does not have reach the live-status formatters below.
                const int idx = reg_.find(cmd.key);
                if (idx >= 0) {
                    char val[settings::MAX_VAL_STR];   // the registry's own cap
                    if (!reg_.valueAsString(idx, val, sizeof(val))) {
                        replyErr(r, "value too long");
                        break;
                    }
                    const int n = std::snprintf(r.text, sizeof(r.text), "%s=%s\n",
                                                cmd.key, val);
                    replyInPlace(r, n > 0 ? static_cast<size_t>(n) : 0);
                } else {
                    getSpecial(r, cmd.key);
                }
            }
            break;

        case Kind::SET: {
            const int idx = reg_.find(cmd.key);
            if (idx >= 0 && loop_.diveActive() && reg_.isDiveLocked(idx)) {
                replyErr(r, "dive locked");
                break;
            }
            const char* serr = nullptr;
            if (!reg_.setValueByKey(cmd.key, cmd.value, &serr)) {
                replyErr(r, serr ? serr : "bad value");
                break;
            }
            r.actions |= ACT_SETTINGS;
            const bool safety = idx >= 0 && (reg_.def(idx).flags & settings::FL_SAFETY);
            replyOk(r, safety ? "SAFETY" : nullptr);
            break;
        }

        case Kind::AUTO:
            if (loop_.requestAuto(true)) replyOk(r, "AUTO");
            else replyErr(r, "predive required");
            break;

        case Kind::MANUAL:
            loop_.requestAuto(false);
            replyOk(r, "MANUAL");
            break;

        case Kind::ACK:
            // Diver outranks the machine. Always honoured, never argued — the
            // machine keeps the condition itself and keeps alarming if it
            // persists; only the latch/silence changes.
            if (safeLockout_) {
                // In SAFE there are no alarms latched in a running chain to
                // silence: here ACK means "I have seen this, get me out", and
                // the transport turns it into a clean restart.
                r.actions |= ACT_SAFE_ACK;
                replyOk(r, "SAFE RESTART");
                break;
            }
            loop_.postAckAlarms();
            replyOk(r);
            break;

        case Kind::CHECK:
            if (loop_.diveActive()) { replyErr(r, "dive active"); break; }
            loop_.postPredive(nowMs_);
            replyOk(r, "CHECK");
            break;

        case Kind::CAL:
            // Refuse what does not exist. Until 2026-11-24 these answered `OK`:
            // `CAL O2` ran a two-step arm/confirm state machine whose consumer
            // (cal::O2Sequencer) is host-tested but never instantiated on the
            // board, and `CAL DEPTH` raised a request flag that nothing read and
            // no depth trim implements. A reply that implies work the machine
            // never did is not a placeholder for a future feature — on a
            // rebreather it is a false all-clear, and the diver has no way to
            // tell it from the real thing. The grammar still parses (so the day
            // the consumer lands no client has to change), and the action bits
            // are retired in cmd_router.h rather than recycled.
            if (cmd.cal == CalAction::DEPTH || cmd.cal == CalAction::O2 ||
                cmd.cal == CalAction::O2_CONFIRM) {
                replyErr(r, "not implemented");
                break;
            }
            if (loop_.diveActive() && cmd.cal != CalAction::NONE) {
                replyErr(r, "cal locked in dive");
                break;
            }
            switch (cmd.cal) {
                // `OK CAL AIR` means the request was accepted, NOT that the
                // cells were calibrated: the warm-up gate and the 12 s window
                // run in ControlTask. That is honest now because the outcome
                // reaches the wire too — `E,<t>,7,<code>` as it happens, and
                // `GET CALAIR` to read it afterwards (bridge::CalAirCode).
                case CalAction::AIR:    r.actions |= ACT_CAL_AIR;      replyOk(r, "CAL AIR"); break;
                default: replyErr(r, "bad cal"); break;
            }
            break;

        case Kind::TESTCELLS:
            if (loop_.cellTestRequested() || loop_.prediveRunning()) {
                replyErr(r, "test running");
                break;
            }
            loop_.requestCellTest();
            replyOk(r, "TEST CELLS");
            break;

        case Kind::CELLPROOF_OVERRIDE:
            // Read to answer, post to act: the override itself runs at the
            // next tick, on the control thread.
            if (loop_.clampActive()) { loop_.postCpOverride(); replyOk(r, "CLAMP LIFTED"); }
            else replyErr(r, "no clamp active");
            break;

        case Kind::PULSE:
            loop_.postTestPulses(1);
            replyOk(r);
            break;

        case Kind::DECO:
            replyInPlace(r, proto::formatDeco(decoInfoOf(loop_, loop_.haveVote()),
                                              r.text, sizeof(r.text)));
            break;

        case Kind::TIME_SET: {
            // Range-checked here, applied by the transport. Deliberately allowed
            // mid-dive: the dive header is written at DIVE START, so setting the
            // clock during a dive cannot rewrite the dive that is running, and a
            // surface PC that has only now noticed the clock is unset should be
            // able to fix it before the NEXT dive rather than be refused.
            if (!wallclock::sane(cmd.epoch)) {
                replyErr(r, "epoch out of range");
                break;
            }
            clockEpoch_ = cmd.epoch;
            r.actions |= ACT_CLOCK_SET;
            replyOk(r, "TIME");
            break;
        }
        case Kind::DIVE_START: startDive(r); break;
        case Kind::DIVE_END:   endDive(r);   break;

        case Kind::LOG_LIST:
            r.actions |= ACT_LOG_LIST;
            replyOk(r);
            break;
        case Kind::LOG_EXPORT:
            logExportIdx_ = cmd.logIndex;
            r.actions |= ACT_LOG_EXPORT;
            replyOk(r);
            break;

        case Kind::WIFI:
            applyWifi(r, std::strcmp(cmd.key, "ON") == 0);
            break;

        case Kind::UNKNOWN:
        default:
            replyErr(r, "unknown command");
            break;
    }
}

} // namespace comms
