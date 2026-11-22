/* ESP-CCR web app — shared runtime.
 * Talks to the firmware over WebSocket (/ws) using the SAME grammar the
 * handset uses (T,/D,/E,/GET/SET). When opened without a device (file://
 * or localhost), it falls back to a built-in dive simulator so the whole
 * UI can be developed and reviewed on a desktop browser.
 */
"use strict";

const App = {
  state: { T: null, D: null, settings: {}, hist: null, events: [], flags: 0,
           mode: "STANDBY", lastRx: 0, ws: null, mock: null },
  listeners: [],

  onChange(fn) { this.listeners.push(fn); },
  emit() { this.listeners.forEach(f => f(this.state)); },

  send(line) {
    if (this.state.ws) this.state.ws.send(line + "\n");
    else this.state.mock.handle(line);
  },

  /* ---- downline grammar parser (mirrors firmware formatters) ---- */
  rx(line) {
    this.state.lastRx = Date.now();
    const s = this.state;
    if (line.startsWith("T,")) {
      const p = line.slice(2).split(",");
      s.T = { depth: +p[0], voted: +p[1], cells: [+p[2], +p[3], +p[4]],
              sp: +p[5], mode: p[6], flags: parseInt(p[7].slice(5), 16) };
      s.mode = p[6]; s.flags = s.T.flags;
      document.body.classList.toggle("alarm", !!(s.flags & 0b111110 & 0b10111));
      document.body.classList.toggle("warn", !!(s.flags & 0b110000));
    } else if (line.startsWith("D,")) {
      const p = line.slice(2).split(",");
      const stops = [];
      for (let i = 3; i < p.length - 1; i++) {
        const [d, t] = p[i].split("|"); stops.push({ depth: +d, min: +t });
      }
      s.D = { ndl: +p[0], total: +p[1], ceiling: +p[2], stops, cns: +p[p.length - 1] };
    } else if (line.startsWith("E,")) {
      const p = line.slice(2).split(",");
      s.events.push({ t: +p[0], id: +p[1], val: +p[2] });
      this.toast(`event ${p[1]} (${p[2]})`);
    } else if (line.startsWith("SETINGS_DUMP") || line.includes("=")) {
      if (!line.startsWith("T,") && !line.startsWith("D,")) {
        line.trim().split("\n").forEach(l => {
          const i = l.indexOf("=");
          if (i > 0) s.settings[l.slice(0, i)] = l.slice(i + 1);
        });
      }
    }
    this.emit();
  },

  toast(msg) {
    const box = document.getElementById("toasts");
    if (!box) return;
    const t = document.createElement("div");
    t.className = "toast"; t.textContent = msg;
    box.appendChild(t);
    setTimeout(() => t.remove(), 5000);
  },

  start() {
    document.body.insertAdjacentHTML("afterbegin", `
      <header>
        <span class="brand">ESP-CCR</span>
        <span class="pill" id="hbmode">BOOT</span>
        <nav>
          <a data-page="status" href="index.html">Status</a>
          <a data-page="histogram" href="histogram.html">Histogram</a>
          <a data-page="checklist" href="checklist.html">Checklist</a>
          <a data-page="settings" href="settings.html">Settings</a>
          <a data-page="dives" href="dives.html">Dives</a>
        </nav>
      </header>
      <div id="stale">DATA LOST — showing last known values</div>
      <div id="toasts"></div>`);
    // try the device; fall back to the built-in simulator
    if (location.protocol.startsWith("http") && !["localhost", "127.0.0.1"].includes(location.hostname)) {
      const ws = new WebSocket(`ws://${location.host}/ws`);
      ws.onopen = () => { this.state.ws = ws; this.send("GET *"); };
      ws.onmessage = e => this.rx(e.data);
      ws.onclose = () => { this.state.ws = null; this.toast("link lost — reconnecting"); setTimeout(() => this.start(), 2000); };
    } else {
      this.state.mock = new MockSim(this);
      this.state.mock.start();
    }
    setInterval(() => {
      const stale = Date.now() - this.state.lastRx > 4000;
      const el = document.getElementById("stale");
      if (el) el.classList.toggle("on", stale);
    }, 1000);
    this.renderChrome();
  },

  renderChrome() {
    const page = document.body.dataset.page;
    document.querySelectorAll("header nav a").forEach(a =>
      a.classList.toggle("on", a.dataset.page === page));
    setInterval(() => {
      const el = document.getElementById("hbmode");
      if (el) el.textContent = this.state.mode || "--";
    }, 500);
  },
};

/* ================= built-in dive simulator (bench/dev only) ============= */
class MockSim {
  constructor(app) {
    this.app = app; this.t = 0; this.phase = 0;
    this.settings = { SP1: "0.400", SP2: "0.700", SP3: "1.000", SP4: "1.300",
      D1: "30.000", D2: "50.000", SP_HYST: "0.500", ALLO_LOW: "0.180",
      ALLO_HIGH: "1.450", ALLO_CUTOFF: "1.600", CNS_ALARM: "80",
      CELL_DRIFT_BAR: "0.050", INJ_PULSE_MS: "300", INJ_MAX_DUTY: "5",
      GF_LOW: "30", GF_HIGH: "85", ASC_RATE: "6.000", ASC_RATE_SLOW: "1.000",
      CELLPROOF_EN: "ON", CELLPROOF_DEPTH: "10.000", CELLPROOF_SP: "1.100",
      CELLPROOF_MIN: "1.000", VBAT_MIN: "10.500", AP_SSID: "ESP-CCR", AP_PASS: "diveonly" };
    this.hist = [new Array(48).fill(0), new Array(48).fill(0), new Array(48).fill(0)];
    this.diveStart = 60; this.diveEnd = 2400;
    app.state.hist = this.hist;
  }
  start() {
    this.app.rx("GET * dump\n" + Object.entries(this.settings).map(([k, v]) => `${k}=${v}`).join("\n"));
    this.iv = setInterval(() => this.tick(), 500);
    setInterval(() => this.sendDeco(), 1000);
  }
  profile(t) {
    if (t < this.diveStart) return 0;
    if (t < this.diveStart + 300) return (t - this.diveStart) / 300 * 40;   // descend
    if (t < this.diveEnd - 900) return 40 + Math.sin(t / 40) * 2;           // bottom
    if (t < this.diveEnd - 300) return Math.max(3, 40 - (t - (this.diveEnd - 900)) / 600 * 37);
    if (t < this.diveEnd) return 3;                                        // stop
    return 0;
  }
  tick() {
    this.t += 0.5;
    const depth = this.profile(this.t);
    const sp = depth < 1.5 ? +this.settings.SP1 : depth < 30 ? +this.settings.SP2 : +this.settings.SP3;
    const base = depth > 0.5 ? sp : 0.21;
    const cells = [base, base + 0.01, base - 0.02 + (this.t > 1500 && this.t < 1700 ? 0.15 : 0)];
    cells.forEach((c, i) => {
      const jitter = c + (Math.random() - 0.5) * 0.01;
      const b = Math.max(0, Math.min(47, Math.round(jitter / 1.9 * 47)));
      this.hist[i][b]++;
    });
    let flags = 0;
    if (this.t > 1500 && this.t < 1700) flags |= 0b100;      // cell drift warning
    if (depth > 0.5 && Math.random() < 0.3) flags |= 0b1000000; // injecting
    const mode = depth > 0.5 ? "AUTO" : (this.t < this.diveStart ? "STANDBY" : "AUTO");
    const T = `T,${depth.toFixed(2)},${base.toFixed(3)},${cells.map(c => c.toFixed(3)).join(",")},${sp.toFixed(3)},${mode},FLAGS0x${flags.toString(16).padStart(2, "0")}`;
    this.app.rx(T);
  }
  sendDeco() {
    const depth = this.profile(this.t);
    const deco = depth > 33;
    const D = deco
      ? `D,0,${(14 + Math.random()).toFixed(1)},12.00,12|3.5,9|4.0,6|6.5,3|3.0,${(20 + this.t / 60).toFixed(1)}`
      : `D,${Math.max(0, Math.round(38 - this.t / 40))},0,0.00,${(10 + this.t / 240).toFixed(1)}`;
    this.app.rx(D);
  }
  handle(line) {
    const m = line.match(/^SET (\S+)=?(.*)$/);
    if (m) {
      if (this.settings[m[1]] === undefined) { this.app.rx("ERR unknown key"); return; }
      this.settings[m[1]] = m[2]; this.app.rx("OK");
    } else if (line.startsWith("GET")) {
      this.app.rx("GET dump\n" + Object.entries(this.settings).map(([k, v]) => `${k}=${v}`).join("\n"));
    } else if (line === "CHECK") {
      this.app.state.checklist && this.app.state.checklist.run();
      this.app.rx("OK");
    } else {
      this.app.rx("ERR unknown command");
    }
  }
}

/* ---------- small shared renderers used by the pages ---------- */
function fmt(v, d = 2) { return (v === null || v === undefined) ? "--" : (+v).toFixed(d); }

function renderFlagPills(T, D) {
  const names = [
    [0b000001, "LOW pO₂", "alarm"], [0b000010, "HIGH pO₂", "alarm"],
    [0b000100, "CELL DRIFT", "warn"], [0b001000, "SENSOR", "alarm"],
    [0b010000, "BATTERY", "warn"], [0b100000, "SP CLAMP", "warn"],
  ];
  return names.filter(([b]) => T && (T.flags & b))
              .map(([, n, c]) => `<span class="pill ${c}">${n}</span>`).join(" ") ||
         '<span class="pill ok">ALL NOMINAL</span>';
}
