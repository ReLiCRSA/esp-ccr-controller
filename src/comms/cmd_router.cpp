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

// The parser has to be able to CARRY what the registry can accept. If either of
// these fires, the two caps have drifted and the board is back to answering `OK`
// for a write it silently shortened: `AP_PASS` published 8..31 characters through
// a 23-character parser field until 2026-11-24, so a diver typing the length the
// device advertised stored a different passphrase than he set (and the endpoint
// fix the same day made those lengths legal rather than refused). The schema a
// handset reads with `GET SCHEMA` is a contract; these asserts are what keep the
// transport from quietly breaking it.
static_assert(sizeof(Command::value) >= settings::MAX_VAL_STR,
              "SET value buffer is shorter than the settings registry's own limit");
static_assert(sizeof(Command::key) >= settings::MAX_KEY,
              "command key buffer is shorter than the settings registry's own limit");

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
        // shownPpo2 == the live vote whenever hasVote is true (loop.cpp step 2b
        // only refreshes the shown number from a healthy vote), so this NDL is
        // computed from a measurement. Without a vote the answer is the protocol's
        // -1: "no number", which is also how a handset knows the ppO2 beside it on
        // `T,` is being held.
        double ndl = z.ndlMinutes(amb, loop.shownPpo2());
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
    // The action is raised on the EFFECTIVE radio state changing, not on the
    // intent changing. A WIFI ON that lands while the DEPTH latch holds the
    // radio sets intent (and is answered OK, truthfully — GET WIFI then shows
    // `WIFI,0,3,0`: wanted on, held by depth) but does not fool the transport
    // into a setRadio(true) the policy would fight a tick later.
    const bool before = wifiRadioWanted();
    wifiOn_ = on;
    // Intent events rewind the idle window, and WIFI ON specifically clears an
    // IDLE hold: "nothing has connected, so we sat down" is exactly the state a
    // diver's WIFI ON is answering. DEPTH is not cleared by words — only by
    // depth. (And WIFI OFF always clears the idle latch: the diver owns the
    // reason the radio is down now.)
    idleHeld_ = false;
    idleSinceMs_ = nowMs_;
    if (!on && loop_.diveActive()) wifiAutoOff_ = false;  // diver's own choice
    if (wifiRadioWanted() != before) r.actions |= CommandRouter::ACT_WIFI;
    replyOk(r);
}

void CommandRouter::startDive(Response& r) {
    if (loop_.diveActive()) { replyErr(r, "dive active"); return; }
    // The dive gate flips here (dive-locked refusals and telemetry mode are
    // true on the very next read); the chain's dive-state work lands at the
    // next control tick — see the post* contract in loop.h.
    loop_.postStartDive(nowMs_);
    logExportIdx_ = 0;
    const bool radioBefore = wifiRadioWanted();
    if (wifiOn_) {            // drop the radio, remember to restore it later
        wifiOn_ = false;
        wifiAutoOff_ = true;
    } else {
        wifiAutoOff_ = false;  // diver had it off: dive end must not turn it on
    }
    idleHeld_ = false;         // the dive owns the reason the radio is down
    if (wifiRadioWanted() != radioBefore) r.actions |= ACT_WIFI;
    r.actions |= ACT_DIVE_START;
    replyOk(r, "DIVE");
}

void CommandRouter::endDive(Response& r) {
    if (!loop_.diveActive()) { replyErr(r, "no dive"); return; }
    loop_.postEndDive(nowMs_);
    r.actions |= ACT_DIVE_END;
    const bool radioBefore = wifiRadioWanted();
    if (wifiAutoOff_) {        // restore what DIVE START took away
        wifiAutoOff_ = false;
        wifiOn_ = true;
    }
    idleHeld_ = false;
    // Deliberate behaviour change of the radio policy (2026-11-26): dive end
    // restores the INTENT, and at depth the DEPTH latch keeps the physical
    // radio down until the diver is below WIFI_ON_DEPTH for WIFI_ON_HOLD_S.
    // A dive ended at 8 m used to bring the AP up at 8 m; that was the very
    // milliamps the dive drop exists to save. On the surface the restore is
    // unchanged, so every existing bench check still sees the AP come back.
    if (wifiRadioWanted() != radioBefore) r.actions |= ACT_WIFI;
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

// `DS,<locked>,<used>,<healthy>,<age_s>,<identity>` — which chip is producing the
// metres, and whether the control path is actually taking them from it.
//
// This exists because "depth: 12.3 m" does not say who measured it. The board can
// carry two depth sensors, the arbiter chooses one at boot by identity and keeps
// it until the next power cycle, and the boot announce states that choice once.
// A client that connects later, or a diver reviewing a machine mid-dive, had no
// way to ask. `<used>` is the field that matters most: a source can be locked and
// identified and still not be feeding the control path (stale bus, no sane
// sample), in which case the metres on the wire are HELD, not measured — which
// reads identically on `T,` and is not visible anywhere else.
//
// `<identity>` is the driver's own string, including the I2C address it was found
// at, and it is copied out under a bound rather than passed to %s directly: the
// driver rewrites that array with snprintf whenever it re-identifies after a bus
// fault, and an array mid-rewrite may have no NUL inside the part copied so far.
// A source that has not been identified never reaches this reply as a name — the
// publisher keeps "none" there.
void CommandRouter::buildDepthSourceLine(Response& r) {
    // The guard that matters: the name is printed ONLY if `locked` vouches for it.
    // A five-field snapshot copied without a lock can always be caught mid-update
    // (see bridge::DepthSource), and the one misleading pair is a name paired with
    // a flag that does not claim a source — which would send a reader to check a
    // chip the arbiter does not own. Printing "none" unless locked is true makes
    // that pair unrepresentable on the wire.
    //
    // The destination is deliberately ROOMIER than the source array (twice the
    // driver's 24 bytes) so that a copy which ignores its bound shows up in the
    // answer instead of quietly overwriting the frame: with a same-size-plus-one
    // destination, an unbounded strcpy is stopped by whatever stale zero happened
    // to be sitting at the end of the local, the reply looks fine, and the
    // corruption is invisible to everything except a crash later. The copy itself
    // is bounded by the SOURCE's declared size, and always terminated.
    char id[2 * bridge::DepthSource::ID_MAX];
    bool locked = false, used = false, healthy = false;
    uint32_t lastGood = 0;
    if (depthSrc_ && depthSrc_->locked) {
        const size_t n = sizeof(depthSrc_->identity) < sizeof(id)
                             ? sizeof(depthSrc_->identity) : sizeof(id);
        std::memcpy(id, depthSrc_->identity, n);
        id[n] = '\0';
        used = depthSrc_->used;
        healthy = depthSrc_->healthy;
        lastGood = depthSrc_->lastGoodMs;
        locked = true;
    } else {
        std::strcpy(id, "none");
    }
    // Age only means something about a source that owns depth and has answered
    // at least once; 0 otherwise, rather than an enormous number derived from
    // "never" (millis 0) that would look like a very old reading.
    const uint32_t age = (locked && lastGood && nowMs_ >= lastGood)
                             ? (nowMs_ - lastGood) / 1000u : 0u;
    char buf[64];
    const int n = std::snprintf(buf, sizeof(buf), "DS,%d,%d,%d,%u,%s\n",
                                locked ? 1 : 0, used ? 1 : 0, healthy ? 1 : 0,
                                static_cast<unsigned>(age), id);
    replyText(r, buf, static_cast<size_t>(n));
}

// ---- radio policy -----------------------------------------------------------
// See cmd_router.h for the rules; this is the machine those rules describe.
// It reads its four WIFI_* settings fresh every tick: SET WIFI_IDLE_MIN=1
// must take effect without a reboot, and four float reads on a task that has
// nothing else to do are free.

int CommandRouter::wifiTick(uint32_t nowMs, bool depthUsed, float depthM,
                            uint8_t stations) {
    wifiStations_ = stations;   // GET WIFI echoes the policy's own input
    // State as this tick FINDS it, before any update: the priming comparison
    // is against reality, not against a post-update snapshot (a controller
    // that booted with the AP up at 5 m has its first down caused by THIS
    // tick — suppressing it because it is the first tick would leave the AP
    // up at depth forever).
    const bool wantedAtEntry = wifiRadioWanted();
    const float offD = reg_.getFloat(reg_.find("WIFI_OFF_DEPTH"));
    const float onD = reg_.getFloat(reg_.find("WIFI_ON_DEPTH"));
    const uint32_t holdS =
        static_cast<uint32_t>(reg_.getFloat(reg_.find("WIFI_ON_HOLD_S")));
    const uint32_t idleM =
        static_cast<uint32_t>(reg_.getFloat(reg_.find("WIFI_IDLE_MIN")));

    // DEPTH tracks whether or not anyone wants a radio: a dive with no DIVE
    // START, and a DIVE END at depth, both need the latch already up when
    // they get here. Stale depth freezes the latch — no transition is
    // inferred from a held number (GET DS says who is believed).
    if (depthUsed) {
        if (!depthHeld_) {
            if (depthM > offD) {
                depthHeld_ = true;
                shallowSinceMs_ = 0;
                idleHeld_ = false;   // a dive is the end of any idle story
                idleSinceMs_ = nowMs;  // and the surface window starts fresh
            }
        } else if (depthM < onD) {
            if (shallowSinceMs_ == 0) shallowSinceMs_ = nowMs;
            if (nowMs - shallowSinceMs_ >= holdS * 1000u) {
                depthHeld_ = false;
                shallowSinceMs_ = 0;
            }
        } else {
            shallowSinceMs_ = 0;  // back inside the hysteresis band: the
                                  // straight-time timer restarts, so a diver
                                  // hovering at 1.5 m is never "shallow"
        }
    }

    // IDLE counts only while the radio is genuinely up — and the window
    // STARTS when the radio becomes up, before any fire check. Both halves
    // matter: a latch that released itself by its own report would flap, and
    // a fire evaluated against the pre-dive window would greet a surfacing
    // diver with "idle" instead of an AP (the old window is dive time, and a
    // 14-minute dive would end with the radio held down by a timer that was
    // never running — the rule under test, found by the test, not by reading).
    const bool upOtherwise = wifiOn_ && !depthHeld_ && !safeLockout_;
    const bool wasUp = wifiLastWanted_;
    if (!upOtherwise || !idleM) {
        // Other holders (or the diver, or SAFE) own the reason the radio is
        // down; and SET WIFI_IDLE_MIN=0 releases a hold its rule no longer
        // arms — leaving the radio down under "never sit down" is a settings
        // lie with an antenna attached.
        idleHeld_ = false;
    } else if (idleHeld_) {
        // stays down until an intent event clears it (applyWifi), never by
        // itself: a self-releasing idle hold flaps the AP once an idle-min forever
    } else if (!wasUp) {
        idleSinceMs_ = nowMs;   // this tick is where "up" begins: nothing has
                                // been idle-counted while the radio was down
    } else if (stations > 0) {
        idleSinceMs_ = nowMs;
    } else if (wifiPrimed_ && nowMs - idleSinceMs_ >= idleM * 60000u) {
        idleHeld_ = true;
    }

    const bool wanted = wifiRadioWanted();
    int change = 0;
    if (!wifiPrimed_) {          // the first tick compares against reality:
        wifiPrimed_ = true;      // surface boot compares equal and prints
        wifiLastWanted_ = wantedAtEntry;   // nothing; depth boot falls through
        idleSinceMs_ = nowMs;    // to the change branch below
    }
    if (wanted != wifiLastWanted_) {
        change = wanted ? 1 : -1;
        wifiLastWanted_ = wanted;   // the idle window restart for an up was
                                    // already done above, BEFORE the fire check
    }
    return change;
}

uint8_t CommandRouter::wifiHold() const {
    if (wifiRadioWanted()) return WH_NONE;
    if (wifiAutoOff_) return WH_DIVE;   // the dive took it: closest reason
    if (!wifiOn_) return WH_DIVER;
    if (depthHeld_) return WH_DEPTH;
    return WH_IDLE;
}

// `WIFI,<on>,<held_by>,<idle_left_s>,<stations>` — the radio's state, the
// reason, and the count behind the reason. held_by is a small enum (see
// WifiHold), not free text: unlike the DS identity, these are states THIS
// machine defines, and a client switches on them (append-only applies: new
// causes append, never renumber).
// idle_left_s: whole seconds until the idle hold would fire, 0 when the
// radio is not up or WIFI_IDLE_MIN is 0 ("never"). A client that wants "can
// I connect?" reads `on`; one that wants "why can't I" reads held_by.
// stations: what the idle rule last saw associated (the driver's count, read
// on loopTask — at most one tick stale). It is the policy's INPUT, published
// for the same reason `used` is: without it, a countdown that never counts
// down is a mystery, and with it the answer is on the line — someone IS
// connected, and the rule refusing to fire is the feature working. 0 when
// the radio is down (there is nothing to associate to).
void CommandRouter::buildWifiLine(Response& r) {
    const bool on = wifiRadioWanted();
    const uint8_t hold = wifiHold();
    uint32_t left = 0;
    if (on) {
        const uint32_t idleM =
            static_cast<uint32_t>(reg_.getFloat(reg_.find("WIFI_IDLE_MIN")));
        if (idleM && wifiPrimed_ && nowMs_ >= idleSinceMs_) {
            const uint32_t tot = idleM * 60u;
            const uint32_t used = (nowMs_ - idleSinceMs_) / 1000u;
            left = used < tot ? tot - used : 0u;
        }
    }
    char buf[48];
    const int n = std::snprintf(buf, sizeof(buf), "WIFI,%d,%u,%u,%u\n",
                                on ? 1 : 0, static_cast<unsigned>(hold),
                                static_cast<unsigned>(left),
                                static_cast<unsigned>(wifiStations_));
    replyText(r, buf, static_cast<size_t>(n));
}

void CommandRouter::getSpecial(Response& r, const char* key) {
    if (std::strcmp(key, "T") == 0) {
        proto::Telemetry t;
        t.depthM = loop_.depthSmoothedM();
        // The shown value, held while the vote is failed (loop.cpp step 2b): a
        // handset must never be handed 0.00 as a measurement. FLAGS bit 3
        // (SENSOR_FAIL/STALE aggregate) is what tells the client it is held.
        t.votedPpo2 = loop_.shownPpo2();
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
    if (std::strcmp(key, "DS") == 0) { buildDepthSourceLine(r); return; }
    if (std::strcmp(key, "WIFI") == 0) { buildWifiLine(r); return; }
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
