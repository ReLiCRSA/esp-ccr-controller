// ============================================================================
// cmd_router.cpp — see cmd_router.h.
//
// The ordering inside exec() is a safety property: parse fully before any
// mutation, and check the dive-lock before the registry check, so a command
// that is refused cannot half-apply. Nothing here touches hardware; the
// transport acts on the returned action bits.
// ============================================================================
#include "cmd_router.h"

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
    if (r.len >= sizeof(r.text)) r.len = sizeof(r.text) - 1;  // snprintf truncates safely
}

void CommandRouter::replyErr(Response& r, const char* why) {
    r.len = proto::formatErr(why, r.text, sizeof(r.text));
}

void CommandRouter::replyText(Response& r, const char* text, size_t n) {
    if (n == 0 || n >= sizeof(r.text)) {
        replyErr(r, "reply overflow");
        return;
    }
    std::memcpy(r.text, text, n);
    r.text[n] = '\0';
    r.len = n;
}

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
    loop_.startDive(nowMs_);
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
    loop_.endDive(nowMs_);
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

void CommandRouter::getSpecial(Response& r, const char* key) {
    if (std::strcmp(key, "T") == 0) {
        proto::Telemetry t;
        t.depthM = loop_.depthSmoothedM();
        t.votedPpo2 = loop_.votedBar();
        for (int i = 0; i < 3; i++) t.cells[i] = loop_.cellBar(i);
        t.setpoint = loop_.setpointBar();
        t.mode = loop_.diveActive() ? (loop_.autoMode() ? 1 : 2) : 0;
        t.flags = telemetryFlags(loop_);
        t.battV = loop_.vbatVolts();
        char buf[128];
        const size_t n = proto::formatTelemetry(t, buf, sizeof(buf));
        replyText(r, buf, n);
        return;
    }
    if (std::strcmp(key, "D") == 0) {
        char buf[CommandRouter::MAX_REPLY];
        const size_t n = proto::formatDeco(decoInfoOf(loop_, loop_.haveVote()), buf, sizeof(buf));
        replyText(r, buf, n);
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
        char buf[64];
        const int n = std::snprintf(buf, sizeof(buf), "FAULT,%d,%d,%.1f\n",
                                    loop_.injector().stuckOpenLatched ? 1 : 0,
                                    loop_.outputFuseTripped() ? 1 : 0,
                                    static_cast<double>(loop_.solDutyPct(nowMs_)));
        replyText(r, buf, static_cast<size_t>(n));
        return;
    }
    // The web Settings page needs each key's type, range and flags to put a
    // min/max on its inputs and grey out what a dive locks; the handset gets
    // the same view. Additive display key — exec() asks the registry before
    // it asks here, so it can never shadow a real setting.
    if (std::strcmp(key, "SCHEMA") == 0) {
        char buf[CommandRouter::MAX_REPLY];
        size_t cnt = 0;
        const settings::Def* defs = settings::Registry::definitions(cnt);
        int p = 0;
        for (size_t i = 0; i < cnt && p < (int)sizeof(buf) - 48; i++) {
            char tc = '?';
            switch (defs[i].type) {
                case settings::Type::FLOAT: tc = 'F'; break;
                case settings::Type::INT16: tc = 'I'; break;
                case settings::Type::UINT8: tc = 'U'; break;
                case settings::Type::BOOL:  tc = 'B'; break;
                case settings::Type::STRING: tc = 'S'; break;
            }
            if (defs[i].type == settings::Type::STRING)
                p += std::snprintf(buf + p, sizeof(buf) - p, "K,%s,%c,%u,%u,%u,%s\n",
                                   defs[i].key, tc, (unsigned)defs[i].minv,
                                   (unsigned)defs[i].maxv, defs[i].flags,
                                   defs[i].defStr ? defs[i].defStr : "");
            else
                p += std::snprintf(buf + p, sizeof(buf) - p, "K,%s,%c,%.6g,%.6g,%u,%.6g\n",
                                   defs[i].key, tc, static_cast<double>(defs[i].minv),
                                   static_cast<double>(defs[i].maxv), defs[i].flags,
                                   static_cast<double>(defs[i].def));
        }
        replyText(r, buf, static_cast<size_t>(p));
        return;
    }

    replyErr(r, "unknown key");
}

// ---- dispatch ----------------------------------------------------------------

CommandRouter::Response CommandRouter::exec(const char* line) {
    Response r{};
    Command cmd;
    const char* err = nullptr;
    if (!proto::parseCommand(line, cmd, &err)) {
        replyErr(r, err ? err : "bad command");
        return r;
    }

    switch (cmd.kind) {
        case Kind::GET:
            if (std::strcmp(cmd.key, "*") == 0) {
                char buf[MAX_REPLY];
                const size_t n = reg_.listAll(buf, sizeof(buf));
                replyText(r, buf, n);
            } else {
                // Registry first: real keys (D1, D2, CELLS_..., CAL_...) must
                // never be shadowed by a display key. Only keys the registry
                // does not have reach the live-status formatters below.
                const int idx = reg_.find(cmd.key);
                if (idx >= 0) {
                    char val[MAX_REPLY];
                    if (!reg_.valueAsString(idx, val, sizeof(val))) {
                        replyErr(r, "reply overflow");
                        break;
                    }
                    char full[MAX_REPLY];
                    const int n = std::snprintf(full, sizeof(full), "%s=%s\n", cmd.key, val);
                    replyText(r, full, static_cast<size_t>(n));
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
            loop_.ackAlarms();
            replyOk(r);
            break;

        case Kind::CHECK:
            if (loop_.diveActive()) { replyErr(r, "dive active"); break; }
            loop_.prediveStart(nowMs_);
            replyOk(r, "CHECK");
            break;

        case Kind::CAL:
            if (loop_.diveActive() && cmd.cal != CalAction::NONE) {
                replyErr(r, "cal locked in dive");
                break;
            }
            switch (cmd.cal) {
                case CalAction::AIR:    r.actions |= ACT_CAL_AIR;      replyOk(r, "CAL AIR"); break;
                case CalAction::O2:
                    // Bare CAL O2 twice = back out. The O2Sequencer itself
                    // (and the enrichment it drives) live in the firmware;
                    // it must see the cancel too, hence the action bit.
                    if (o2Armed_) { o2Armed_ = false; r.actions |= ACT_CAL_O2_CANCEL;
                                    replyOk(r, "CAL O2 CANCEL"); }
                    else { o2Armed_ = true; r.actions |= ACT_CAL_O2_ARM;
                           replyOk(r, "CAL O2 ARMED"); }
                    break;
                case CalAction::O2_CONFIRM:
                    if (!o2Armed_) { replyErr(r, "not armed"); break; }
                    o2Armed_ = false;
                    r.actions |= ACT_CAL_O2_CONFIRM;
                    replyOk(r, "CAL O2");
                    break;
                case CalAction::DEPTH:  depthCalPending_ = true;
                                        r.actions |= ACT_CAL_DEPTH;    replyOk(r, "CAL DEPTH"); break;
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
            if (loop_.cellproofOverride()) replyOk(r, "CLAMP LIFTED");
            else replyErr(r, "no clamp active");
            break;

        case Kind::PULSE:
            loop_.requestTestPulse();
            replyOk(r);
            break;

        case Kind::DECO: {
            char buf[MAX_REPLY];
            const size_t n = proto::formatDeco(decoInfoOf(loop_, loop_.haveVote()), buf, sizeof(buf));
            replyText(r, buf, n);
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
    return r;
}

} // namespace comms
