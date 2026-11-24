// ============================================================================
// web_server.cpp — AP + WebSocket + static pages (PLAN.md 9).
//
// Why it is shaped this way:
//   * A frame in is one handset protocol line through comms::session(). There
//     is no JSON command layer, no REST resource for "set the setpoint": a
//     browser reaches the identical CommandRouter the handset does, so the
//     pre-dive AUTO lockout, the dive-locked settings and the two-step CAL O2
//     confirm cannot be stepped around from a phone.
//   * Everything socket-facing stays in the async task's own callback. No
//     other task ever touches a socket, so the library's client objects are
//     only ever used from one place.
//   * The radio is turned off physically (softAPdisconnect + WIFI_OFF), never
//     by "closing the server": PLAN 9 counts the milliamps and the antenna.
//   * Sockets are NOT closed from the command path. A web client that sends
//     DIVE START must not have its own connection destroyed inside its own
//     callback (that is a use-after-free in most async stacks); the radio
//     going down kills the connections on its own schedule, and start() runs
//     cleanupClients() afterwards.
//
// Framing (WebSocket only, documented in PLAN 9): the protocol is line-based
// and some commands answer with many lines (GET *, LOG EXPORT, DECO), so the
// bridge ends every command with a control line "#<n>" — n reply lines
// followed. A browser can therefore know an answer is complete without
// guessing. '#' can never begin a protocol line, so nothing collides.
// ============================================================================
#include "comms/web_server.h"

#include <Arduino.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "comms/session.h"
#include "comms/settings.h"
#include "comms/uart_proto.h"
#include "web_page.h"

extern settings::Registry g_settings;

namespace comms {
namespace web {

namespace {

AsyncWebServer* g_server = nullptr;
AsyncWebSocket* g_ws = nullptr;
DNSServer* g_dns = nullptr;
bool g_on = false;
uint32_t g_minHeap = 0xFFFFFFFFu;
uint32_t g_framesIn = 0, g_framesOut = 0;

/// Per-client receive buffer: command lines are short, and a fragmenting or
/// overlong client cannot make this grow. Allocated on connect, freed on
/// disconnect (the library does not manage _tempObject for WS clients).
/// The verdict — an over-long line is DROPPED WHOLE, never truncated-and-run —
/// is the same one the UART path takes via `proto::LineReader` (see PLAN 16);
/// a WebSocket frame arrives whole, so "whole" is decided here by a flag rather
/// than by swallowing to the next terminator.
struct ClientLine {
    size_t len = 0;
    bool overflow = false;
    char text[proto::MAX_LINE + 2];
};

void noteHeap() {
    const uint32_t f = ESP.getFreeHeap();
    if (f < g_minHeap) g_minHeap = f;
}

// ---- the sink: protocol lines out to one client ----------------------------

/// Reply batching. The link hands over whole replies — GET SCHEMA is 32
/// protocol lines in one call — and one WebSocket frame per line used to put
/// 33 tiny TCP segments on the wire at once. On the bench that burst stalls
/// AsyncTCP's output for seconds (observed: a >13 s hole mid-telemetry after
/// a SCHEMA), and the page correctly calls any silence over 3 s "NO DATA".
/// So lines accumulate into whole frames (the page splits frames on newlines
/// anyway — protocol unchanged), and the marker after each reply flushes the
/// batch. Frames per SCHEMA: ~2, not 33.
struct SinkCtx {
    uint32_t id;
    uint16_t lines = 0;     // PROTOCOL lines emitted (the marker's count)
    char batch[1024];
    size_t blen = 0;

    void flush() {
        if (!blen) return;
        g_ws->text(id, batch, blen);
        g_framesOut++;
        blen = 0;
    }
    void add(const char* data, size_t n) {
        if (!n) return;
        if (blen + n > sizeof(batch)) flush();
        if (n > sizeof(batch)) {          // pathological single line: own frame
            g_ws->text(id, data, n);
            g_framesOut++;
            return;
        }
        std::memcpy(batch + blen, data, n);
        blen += n;
    }
};

void wsSink(const char* text, void* vctx) {
    if (!g_ws || !text || !*text) return;
    SinkCtx& s = *static_cast<SinkCtx*>(vctx);
    const char* p = text;
    while (*p) {
        const char* nl = std::strchr(p, '\n');
        const size_t n = nl ? static_cast<size_t>(nl - p) : std::strlen(p);
        if (n > 0) {
            s.add(p, n);
            s.add("\n", 1);
            s.lines++;
        }
        if (!nl) break;
        p = nl + 1;
        while (*p == '\r') p++;
    }
}

void sendStatusMarker(uint32_t id, uint16_t lines, bool dropped) {
    if (!g_ws) return;
    char marker[32];
    const size_t n = formatReplyMarker(marker, sizeof(marker), lines, dropped);
    if (n) g_ws->text(id, marker, n);
}

void runCommand(uint32_t id, const char* line, size_t len, bool overflow) {
    g_framesIn++;
    SinkCtx out;
    out.id = id;
    if (overflow) {
        char buf[48];
        const size_t n = proto::formatErr("line too long", buf, sizeof(buf));
        if (n) {
            buf[n] = '\0';
            g_ws->text(id, buf, n);
            out.lines++;
        }
        sendStatusMarker(id, out.lines, true);
        return;
    }
    // The shared session: same router, same lockouts, same dive file owner as
    // the handset. millis() because dive edges must land on the chain's clock.
    session().handleLine(line, millis(), wsSink, &out);
    out.flush();   // the reply's tail leaves as one frame, then the marker
    sendStatusMarker(id, out.lines, false);
    noteHeap();
}

// ---- WebSocket plumbing ----------------------------------------------------

void onWsEvent(AsyncWebSocket* /*server*/, AsyncWebSocketClient* client, AwsEventType type,
               void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT: {
            ClientLine* cl = new (std::nothrow) ClientLine();
            client->_tempObject = cl;
            noteHeap();
            // Greeting carries the clock and heap: the UI shows both, and a
            // phone that connects after a reboot sees that immediately.
            char hello[64];
            const int n = std::snprintf(hello, sizeof(hello), "#boot %lu %lu\n",
                                        (unsigned long)millis(),
                                        (unsigned long)ESP.getFreeHeap());
            if (n > 0) g_ws->text(client->id(), hello, static_cast<size_t>(n));
            return;
        }
        case WS_EVT_DISCONNECT: {
            delete static_cast<ClientLine*>(client->_tempObject);
            client->_tempObject = nullptr;
            return;
        }
        case WS_EVT_ERROR:
        case WS_EVT_PING:
        case WS_EVT_PONG:
            return;

        case WS_EVT_DATA: {
            AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
            if (info->opcode != WS_TEXT && info->message_opcode != WS_TEXT) return;

            ClientLine* cl = static_cast<ClientLine*>(client->_tempObject);
            if (cl == nullptr) {
                cl = new (std::nothrow) ClientLine();
                client->_tempObject = cl;
                if (cl == nullptr) return;   // no buffer: ignore, do not crash
            }
            if (info->index == 0) {  // first fragment (or the whole message)
                cl->len = 0;
                cl->overflow = false;
            }
            for (size_t i = 0; i < len; i++) {
                const char c = static_cast<char>(data[i]);
                if (c == '\n' || c == '\r') break;   // one command per frame
                if (cl->len < sizeof(cl->text) - 1) cl->text[cl->len++] = c;
                else cl->overflow = true;
            }
            cl->text[cl->len] = '\0';

            const bool complete = info->final && (info->index + len >= info->len);
            if (!complete) return;
            if (cl->len == 0 && !cl->overflow) return;   // blank frame: ignore
            runCommand(client->id(), cl->text, cl->len, cl->overflow);
            cl->len = 0;
            cl->overflow = false;
            return;
        }
    }
}

// ---- static pages ---------------------------------------------------------

bool assetInstalled(const char* path) {
    // Never mount from a network callback: LittleFS is owned by the dive log.
    // If it is not mounted the answer is simply "not installed".
    return LittleFS.exists(path);
}

void serveIndex(AsyncWebServerRequest* req) {
    // The page always comes out of app flash, never out of LittleFS. That used
    // to be the fallback and the filesystem copy won whenever one was installed —
    // and once the dive log mounts at boot, it always is. Two ways that loses: an
    // installed copy is a generation behind the firmware that serves it (the page
    // a diver reads is not the page this build was tested with), and the only way
    // to refresh it is `pio run -t uploadfs`, which rewrites the partition that the
    // dive files live on — erasing dive history to update a stylesheet. The
    // built-in copy ships in the firmware image, so it cannot be stale, cannot be
    // half-written, and costs the diver nothing when the filesystem is broken.
    //
    // Phone browsers cache aggressively; a stale cached page after a firmware
    // fix is an "I fixed it but nothing changed" bug report every time.
    AsyncWebServerResponse* rsp = req->beginResponse_P(200, "text/html", kIndexHtml);
    rsp->addHeader("Cache-Control", "no-store");
    req->send(rsp);
}

void serveAsset(AsyncWebServerRequest* req, const char* path, const char* type) {
    // Optional escape hatch for a file someone installs deliberately (the
    // built-in page references nothing, so nothing here is required: /style.css
    // and /app.js answer 404 on a normal board, and `web_probe.py` asserts that).
    if (assetInstalled(path)) req->send(LittleFS, path, type);
    else req->send(404, "text/plain", "not installed");
}

void serveFavicon(AsyncWebServerRequest* req) { req->send(204); }

// ---- dive CSV download -----------------------------------------------------
// The browser wants a file, not a frame stream, so this reads the same dive
// file through the same session lock as `LOG EXPORT <n>` and answers once.

struct CsvCollect {
    char* buf;
    size_t cap;
    size_t len;
};

void collectLine(const char* line, void* vctx) {
    CsvCollect& c = *static_cast<CsvCollect*>(vctx);
    const size_t n = std::strlen(line);
    if (c.len + n < c.cap) {
        std::memcpy(c.buf + c.len, line, n);
        c.len += n;
        c.buf[c.len] = '\0';
    }
    // Overflow is deliberately silent-but-stopping: the CSV is already
    // truncated and a half-written row would be worse than a short file.
}

void serveDiveCsv(AsyncWebServerRequest* req, int slot) {
    if (slot < 1 || slot > 10) {
        req->send(400, "text/plain", "dive number 1..10");
        return;
    }
    // ~15 KB per hour of dive by design (PLAN 10); 64 KB is generous and this
    // runs only when a human asks for a file at the surface.
    constexpr size_t kCap = 64 * 1024;
    char* buf = static_cast<char*>(std::malloc(kCap));
    if (!buf) {
        req->send(503, "text/plain", "no memory for export");
        return;
    }
    CsvCollect c{buf, kCap, 0};
    buf[0] = '\0';
    const int records = diveLogExportCsv(slot, collectLine, &c);
    if (records < 0) {
        std::free(buf);
        req->send(404, "text/plain", "no such dive");
        return;
    }
    char name[24];
    std::snprintf(name, sizeof(name), "dive%d.csv", slot);
    AsyncWebServerResponse* r = req->beginResponse(200, "text/csv", buf);
    r->addHeader("Content-Disposition", String("attachment; filename=") + name);
    req->send(r);
    std::free(buf);
    noteHeap();
}

void onNotFound(AsyncWebServerRequest* req) {
    // Phones type half a URL; send them to the page rather than an error.
    req->redirect("/");
}

} // namespace

bool start() {
    if (g_on) return true;

    const int ssidIdx = g_settings.find("AP_SSID");
    const int passIdx = g_settings.find("AP_PASS");
    const String base = ssidIdx >= 0 ? String(g_settings.getString(ssidIdx)) : String("ESP-CCR");
    const String pass = passIdx >= 0 ? String(g_settings.getString(passIdx)) : String("diveonly");

    WiFi.persistent(false);   // a soft AP must never wear flash for a reconnect
    WiFi.mode(WIFI_AP);       // must precede the MAC read below

    // Suffix with the MAC tail: on a bench with three boards, "which AP is
    // mine" must be answerable without a label, and a phone that joined once has
    // to be able to rejoin after every dive.
    //
    // Read after WiFi.mode() because that is the order that is defined to work,
    // and remember it once so the name cannot drift. Measured on the bench
    // 2026-11-23 rather than assumed: reading the MAC before or after the mode
    // call makes NO observable difference, because WiFi.macAddress() returns the
    // core's cached address without touching a stopped driver. So this is not
    // what causes "esp_wifi_get_mac failed with 12289" on the console — that
    // line is ESP-IDF's own read inside esp_wifi_stop(); the Arduino WiFi library
    // never calls esp_wifi_get_mac at all.
    static String tail;
    if (tail.length() == 0) {
        const String mac = WiFi.macAddress();      // "AA:BB:CC:DD:EE:FF"
        if (mac.length() >= 5) {
            String hex = mac.substring(mac.length() - 5);
            hex.replace(":", "");
            tail = "-" + hex;
        }
    }
    const String ssid = base + tail;
    if (!WiFi.softAP(ssid, pass, 1, false, 4)) {
        WiFi.mode(WIFI_OFF);
        Serial.println("** web: softAP failed");
        return false;
    }

    // Captive DNS: a joined phone resolves NOTHING until something answers
    // port 53, so typing any name fails and only a raw IP would work. Answer
    // every query with the AP itself; onNotFound() then turns the visit into
    // the page. One query per service() poll — the handset task's 20 ms loop
    // keeps that responsive, and the radio is off during dives anyway.
    if (g_dns == nullptr) g_dns = new (std::nothrow) DNSServer();
    if (g_dns) {
        g_dns->setErrorReplyCode(DNSReplyCode::NoError);
        // "*" is the library's wildcard token — an empty domain does NOT
        // match and answers come back empty (verified on the wire).
        if (!g_dns->start(53, "*", WiFi.softAPIP())) {
            Serial.println("** web: captive DNS failed to bind :53");
            delete g_dns;
            g_dns = nullptr;
        }
    }

    if (g_server == nullptr) {
        g_server = new (std::nothrow) AsyncWebServer(80);
        g_ws = new (std::nothrow) AsyncWebSocket("/ws");
        if (g_server == nullptr || g_ws == nullptr) {
            Serial.println("** web: server allocation failed");
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_OFF);
            return false;
        }
        g_ws->onEvent(onWsEvent);
        g_server->addHandler(g_ws);
        g_server->on("/", HTTP_GET, serveIndex);
        g_server->on("/style.css", HTTP_GET, [](AsyncWebServerRequest* r) {
            serveAsset(r, "/style.css", "text/css");
        });
        g_server->on("/app.js", HTTP_GET,
                     [](AsyncWebServerRequest* r) { serveAsset(r, "/app.js", "text/javascript"); });
        g_server->on("/favicon.ico", HTTP_GET, serveFavicon);
        g_server->on("/dive/1.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 1); });
        g_server->on("/dive/2.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 2); });
        g_server->on("/dive/3.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 3); });
        g_server->on("/dive/4.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 4); });
        g_server->on("/dive/5.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 5); });
        g_server->on("/dive/6.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 6); });
        g_server->on("/dive/7.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 7); });
        g_server->on("/dive/8.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 8); });
        g_server->on("/dive/9.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 9); });
        g_server->on("/dive/10.csv", HTTP_GET, [](AsyncWebServerRequest* r) { serveDiveCsv(r, 10); });
        g_server->onNotFound(onNotFound);
    }
    g_server->begin();
    g_ws->cleanupClients();   // sockets that died with the radio on the last dive
    g_on = true;
    noteHeap();
    Serial.printf("web: AP \"%s\" on %s (heap %lu)\n", ssid.c_str(),
                  WiFi.softAPIP().toString().c_str(), (unsigned long)ESP.getFreeHeap());
    return true;
}

void stop() {
    if (!g_on) return;
    // No closeAll() here: the command path may be inside a client's own
    // callback, and destroying a socket underneath its own handler is how
    // async stacks get use-after-frees. The radio going down ends the
    // connections; start() cleans the leftovers up.
    if (g_dns) {
        g_dns->stop();
        delete g_dns;
        g_dns = nullptr;  // :53 belongs to nobody while the radio is down
    }
    WiFi.softAPdisconnect(true);
    // Drain, then stop. The deauth of any associated station is asynchronous:
    // the driver's worker (ppTask) is still inside an ioctl when this returns.
    // esp_wifi_stop() across that window asserts inside the driver —
    // `spinlock_acquire` in semphr_give_wrapper — and that is a reboot, not a
    // warning. Observed on the bench 2026-11-23 from the dive-start radio drop.
    // The pause belongs here rather than in the caller: this runs on loopTask,
    // which has no control duty, and never in the task answering a client.
    delay(250);
    WiFi.mode(WIFI_OFF);
    g_on = false;
    // Deliberately nothing here touches g_ws. Its client list keeps the dead
    // sockets until start() runs cleanupClients() on the way back up, and
    // AsyncWebSocket::enable() is NOT a way to close them (in 3.12.1 it only
    // gates canHandle(), i.e. whether a new upgrade is accepted), so a call to
    // it here would be a no-op dressed up as teardown.
    Serial.println("web: radio off (dive)");
}

// Radio intent is state; the action belongs on loopTask. See web_server.h.
volatile bool g_radioWanted = true;
volatile bool g_radioChange = false;

void setRadio(bool on) {
    g_radioWanted = on;
    g_radioChange = true;
}

bool radioPending() { return g_radioChange && (g_radioWanted != g_on); }

void serviceRadio() {
    if (!g_radioChange) return;
    g_radioChange = false;
    if (g_radioWanted == g_on) return;
    if (g_radioWanted) start();
    else stop();
}

void service() {
    // Polled by the handset task (its loop is the firmware's one general
    // 20 ms poller). Non-blocking: parsePacket() only moves a datagram if
    // one is actually waiting, so an idle radio costs one select here.
    if (g_dns) g_dns->processNextRequest();
}

bool isOn() { return g_on; }

size_t clientCount() { return g_ws ? g_ws->count() : 0; }

uint32_t minFreeHeap() { return g_minHeap; }

} // namespace web
} // namespace comms
