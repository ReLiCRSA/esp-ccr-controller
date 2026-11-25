// ============================================================================
// web_page.h — the status/settings UI (PLAN.md 9), and the ONLY copy the
// controller serves. It lives in app flash next to the code it talks to.
//
// Plain HTML/CSS/vanilla JS, one file, no framework and no CDN: the access
// point has no internet, ever, and a page that cannot be fetched is a page
// that cannot be read on the day it matters. Keeping it in the firmware image
// also means the worst LittleFS outcome (not mounted, half-written, wiped, or
// holding a copy of the previous generation) costs the UI nothing — the phone
// still gets the screen this build was tested with.
//
// The same text is extracted to data/index.html (tools/sync_webdata.py) as the
// reference copy: `tools/web_probe.py` fetches / over the AP and requires the
// bytes to match, which is what keeps the two in step. It is not an install
// target — installing it means `uploadfs`, which rewrites the partition the dive
// logs share and erases dive history.
//
// Protocol: every command line is answered with reply lines and then a
// '#<n>' control line (n = number of reply lines), so the page knows when an
// answer is complete. See web_server.cpp.
// ============================================================================
#pragma once

#include <pgmspace.h>

// clang-format off
static const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html>
<!-- Served out of app flash, byte for byte. The filesystem copy of this page is
     never used: updating it would mean uploadfs, and uploadfs erases the dive
     logs on the same partition. -->
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="theme-color" content="#0b0f14">
<title>ESP-CCR</title>
<style>
:root{--bg:#0b0f14;--panel:#141a22;--ink:#e8eef6;--dim:#8c9bb0;--line:#25303d;
--ok:#39d98a;--warn:#ffb020;--bad:#ff4d4d;--accent:#39d98a}
body.alarm{--accent:#ff4d4d} body.warn{--accent:#ffb020} body.stale{--accent:#5b6b7d}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.45 system-ui,sans-serif;
border-top:5px solid var(--accent);transition:border-color .3s}
header{padding:10px 12px 0;border-bottom:1px solid var(--line)}
#brand{font-weight:700;letter-spacing:.06em;color:var(--accent)}
#sysline{color:var(--dim);font-size:13px;margin-top:2px}
nav{display:flex;gap:4px;margin-top:10px;flex-wrap:wrap}
nav button{background:var(--panel);color:var(--dim);border:1px solid var(--line);
border-bottom:none;border-radius:8px 8px 0 0;padding:10px 14px;min-height:44px;font-size:15px}
nav button.on{color:var(--ink);background:var(--bg);font-weight:600}
main{padding:14px} section{display:none} section.on{display:block}
#stale{background:var(--bad);color:#fff;padding:8px 12px;font-weight:600}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.hero{display:flex;align-items:baseline;gap:12px;flex-wrap:wrap}
.hero .v{font-size:64px;font-weight:800;line-height:1;font-variant-numeric:tabular-nums}
.hero .u{color:var(--dim);font-size:18px}
.tgt{color:var(--dim);font-size:15px}
.row{display:flex;justify-content:space-between;gap:10px;padding:6px 0;border-bottom:1px solid var(--line)}
.row:last-child{border-bottom:none}
.k{color:var(--dim)}.n{font-variant-numeric:tabular-nums;font-weight:600}
.cells{display:grid;gap:8px}
.cell{display:grid;grid-template-columns:44px 1fr 74px;align-items:center;gap:10px}
.bar{height:14px;background:#0e141b;border:1px solid var(--line);border-radius:7px;overflow:hidden}
.bar i{display:block;height:100%;background:var(--accent)}
.bar i.off{background:#3d4b5c}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(230px,1fr))}
.pill{display:inline-block;padding:4px 9px;border-radius:999px;font-size:13px;font-weight:600;
border:1px solid var(--line);color:var(--dim);margin:2px 3px 2px 0}
.pill.on{color:#0b0f14;border-color:transparent}
.pill.on.ok{background:var(--ok)}.pill.on.warn{background:var(--warn)}.pill.on.bad{background:var(--bad)}
button.act{background:#1b2530;color:var(--ink);border:1px solid var(--line);border-radius:10px;
padding:12px 14px;min-height:44px;font-size:15px;margin:3px 4px 3px 0}
button.act.big{background:var(--accent);color:#0b0f14;font-weight:700;border-color:transparent}
button.act.danger{background:#3a1618;border-color:#6b2a2e;color:#ffd9db}
input,select{background:#0e141b;color:var(--ink);border:1px solid var(--line);border-radius:9px;
padding:11px 12px;min-height:44px;font-size:15px;width:100%}
fieldset{border:1px solid var(--line);border-radius:12px;padding:12px;margin:0 0 12px}
legend{color:var(--accent);font-weight:700;padding:0 6px;font-size:14px}
.set{display:grid;grid-template-columns:1fr 132px;gap:10px;align-items:center;padding:6px 0}
.set .lab{font-size:14px}.set .rng{color:var(--dim);font-size:12px}
.set.locked .lab::after{content:" \1F512";color:var(--warn);font-size:12px}
#log{background:#0e141b;border:1px solid var(--line);border-radius:10px;padding:10px;
font:12px/1.5 ui-monospace,monospace;white-space:pre-wrap;min-height:120px;max-height:46vh;overflow:auto;margin:0}
form.cmd{display:flex;gap:8px;margin-top:10px}
form.cmd button{min-width:88px}
#toasts{position:fixed;left:50%;transform:translateX(-50%);bottom:16px;display:grid;gap:8px;
z-index:9;width:min(94vw,520px)}
.toast{background:#1b2530;border-left:4px solid var(--accent);padding:11px 13px;border-radius:9px;
box-shadow:0 6px 22px rgba(0,0,0,.5);font-size:14px}
.toast.bad{border-left-color:var(--bad)}.toast.ok{border-left-color:var(--ok)}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:8px 6px;border-bottom:1px solid var(--line);text-align:left}
th{color:var(--dim);font-weight:600;font-size:13px}
a{color:var(--accent)}
canvas{width:100%;height:180px;background:#0e141b;border:1px solid var(--line);border-radius:10px}
.hist{display:flex;align-items:flex-end;gap:1px;height:130px;margin:6px 0 2px}
.hist i{flex:1;background:var(--accent);opacity:.85;min-height:2px}
.hist i.z{visibility:hidden}
.hint{color:var(--dim);font-size:13px}
.verdicts{margin-top:8px}
.vrow{font-size:13px;padding:2px 8px;border-left:3px solid var(--dim);margin:2px 0}
.vrow.bad{border-left-color:#ff5b5b}.vrow.warn{border-left-color:#ffb020}.vrow.ok{border-left-color:#39d98a}
@media(min-width:900px){main{display:grid;grid-template-columns:2fr 1fr;gap:14px;align-items:start}}
</style>
</head>
<body class="clean">
<header>
  <div id="brand">ESP-CCR</div>
  <div id="sysline"><span id="link">connecting&#8230;</span> <span id="clock"></span> <span id="wclock"></span></div>
  <nav id="tabs">
    <button data-v="status" class="on">Status</button>
    <button data-v="check">Checklist</button>
    <button data-v="hist">Cells</button>
    <button data-v="settings">Settings</button>
    <button data-v="dives">Dives</button>
    <button data-v="surface">Surface</button>
    <button data-v="console">Console</button>
  </nav>
</header>
<div id="stale" hidden>NO DATA &#8212; the radio is off in a dive, or the link is down</div>
<main>
<section id="v-status" class="on">
  <div class="card">
    <div class="hero"><span class="v" id="ppo2">--.-</span><span class="u">bar ppO&#8322;</span>
      <span class="tgt">target <b id="sp">-.-</b></span></div>
    <div id="flags" style="margin-top:10px"></div>
  </div>
  <div class="card"><div class="cells" id="cells"></div>
    <div class="hint" id="proofline"></div></div>
  <div class="card">
    <div class="row"><span class="k">depth</span><span class="n" id="depth">-</span></div>
    <div class="row"><span class="k">ceiling / NDL</span><span class="n" id="cn">-</span></div>
    <div class="row"><span class="k">deco</span><span class="n" id="deco">-</span></div>
    <div class="row"><span class="k">CNS O&#8322;</span><span class="n" id="cns">-</span></div>
    <div class="row"><span class="k">battery</span><span class="n" id="bat">-</span></div>
    <div class="row"><span class="k">injection</span><span class="n" id="inj">-</span></div>
    <div class="row"><span class="k">radio</span><span class="n" id="radio">-</span></div>
  </div>
</section>
<section id="v-check">
  <div class="card"><div id="pre"></div><div class="hint" id="cellline"></div>
    <div class="hint" id="calline"></div></div>
  <div class="card">
    <button class="act big" data-c="CHECK">Run pre-dive checks</button>
    <button class="act" data-c="TEST CELLS">Test cells</button>
    <button class="act" data-c="AUTO">AUTO</button>
    <button class="act" data-c="MANUAL">MANUAL</button>
    <button class="act" data-c="ACK">ACK alarms</button>
  </div>
  <div class="card">
    <button class="act big" data-c="DIVE START">DIVE START</button>
    <button class="act danger" data-c="DIVE END">DIVE END</button>
    <button class="act" data-c="GET PRE">Refresh checks</button>
    <button class="act danger" data-c="CELLPROOF OVERRIDE">Lift clamp</button>
  </div>
  <div class="card"><fieldset><legend>Calibration</legend>
    <div class="hint">Air is mandatory and runs on the board: press CAL AIR, then read the
      verdict below &#8212; it says whether the cells were sampled, refused while still
      warming up, or failed, and with what raw numbers. The O&#8322; point (a sealed loop
      enriched to <span id="o2fi">0.99</span>) and the depth tare are <b>not implemented in
      this firmware</b>: asking them answers &#8220;not implemented&#8221;, so no button here
      pretends otherwise.</div>
    <button class="act" data-c="CAL AIR">CAL AIR</button>
    <button class="act" data-c="GET CALAIR">CAL AIR status</button>
  </fieldset></div>
</section>
<section id="v-hist">
  <div class="card"><div id="h0"><span class="hint">no cell history yet</span></div></div>
  <div class="card"><div id="h1"></div></div>
  <div class="card"><div id="h2"></div></div>
  <div class="hint" id="hline"></div>
</section>
<section id="v-settings"><div id="sets"></div></section>
<section id="v-dives">
  <div class="card"><button class="act big" id="refresh">List dives</button>
    <div class="hint">Ten most recent dives, oldest overwritten.</div></div>
  <div class="card" id="divelist"></div>
  <div class="card" id="diveview" hidden></div>
</section>
<section id="v-surface">
  <div class="card"><button class="act big" id="surfreq">Load surface log</button>
    <a class="act" style="display:inline-block;padding:12px" href="/surface.csv">download csv</a>
    <div class="hint">Every calibration verdict and pre-dive result the controller has
      recorded, newest first. These are not dive rows: a pre-dive failure prevents the
      dive, so there is no dive file to put it in, and a calibration verdict is a fact
      about the machine. A wired handset has no command for this history &#8212; it can
      only be listening when the verdict happens &#8212; so this page is the only way to
      read it back.</div></div>
  <div class="card" id="surflog"><span class="hint">not loaded yet</span></div>
</section>
<section id="v-console">
  <div class="card"><pre id="log"></pre>
    <form class="cmd" id="cmdform"><input id="cline" autocomplete="off" spellcheck="false"
      placeholder="GET T"><button class="act">send</button></form>
    <div class="hint">Everything here is the handset protocol, byte for byte.</div></div>
</section>
</main>
<div id="toasts"></div>
<script>
"use strict";
var $=function(id){return document.getElementById(id)};
var view="status", sock=null, outq=[], busy=null, lastRx=0, bootMs=0, tries=0;
var T=null, D=null, CELLS=null, PROOF=null, PRE=null, FAULT=null, CALAIR=null, WIFI=null;
var SCHEMA=[], VALS={}, diveActive=false;

function toast(msg,kind){
  var d=document.createElement("div"); d.className="toast"+(kind?" "+kind:""); d.textContent=msg;
  $("toasts").appendChild(d);
  setTimeout(function(){ if(d.parentNode) d.parentNode.removeChild(d); },5200);
}
function logline(s){
  var p=$("log"); p.textContent+=s+"\n";
  var lines=p.textContent.split("\n");
  if(lines.length>400) p.textContent=lines.slice(lines.length-400).join("\n");
  p.scrollTop=p.scrollHeight;
}

// ---- the command pipe: one outstanding command, '#' ends the answer --------
function send(line){
  return new Promise(function(res){ outq.push({l:line,lines:[],res:res}); pump(); });
}
function pump(){
  if(busy||!sock||sock.readyState!==1) return;
  var c=outq.shift(); if(!c) return;
  busy=c; logline("> "+c.l); sock.send(c.l);
  setTimeout(function(){ if(busy===c){ finish("ERR reply timeout"); } },4000);
}
function finish(extra){
  if(extra) { busy.lines.push(extra); }
  var c=busy; busy=null; if(c) c.res(c.lines);
  pump();
}
// ---- wall clock ------------------------------------------------------------
// The controller has no RTC chip and the access point has no uplink, so the
// phone's own clock IS the dive log's time source: this page posts it every
// time the socket opens. The board carries that time across its own restarts
// (panic, watchdog), but an EN cycle or a battery pull loses it - which is
// exactly why the page posts on every open instead of relying on the carry.
// The window guard is the firmware's own accepted range (2000..2100): a
// phone whose clock is wilder than that must not make the log worse, and an
// honest "no clock" beats a confident wrong date.
var WCLK=null;
function syncClock(){
  var ep=Math.floor(Date.now()/1000);
  if(ep<946684800||ep>4102444800){
    toast("phone clock out of range: dives will carry no date","bad");
    return;
  }
  send("TIME SET "+ep).then(function(ls){
    if(ls.some(function(l){return l.indexOf("ERR")===0;}))
      toast("controller refused the phone clock","bad");
    readClock();
  });
}
function readClock(){
  send("GET TIME").then(function(ls){
    var t=ls.filter(function(l){return l.indexOf("TIME,")===0;});
    if(t.length){ WCLK=t[0].split(","); showClock(); }
  });
}
function showClock(){
  var el=$("wclock"); if(!el) return;
  if(!WCLK){ el.textContent=""; return; }
  var ep=+WCLK[1], src=+WCLK[2];
  // TIME,<epoch>,<source>,<age_s>: source 2 means the number survived a reset
  // without anyone confirming it since, which is worth saying out loud.
  el.textContent = ep>0
    ? "clock "+new Date(ep*1000).toISOString().slice(0,16).replace("T"," ")+"Z"+(src===2?" (carried)":"")
    : "clock UNSET: dives will carry no date";
}
function connect(){
  var url=(location.protocol==="https:"?"wss":"ws")+"://"+location.host+"/ws";
  sock=new WebSocket(url);
  sock.onopen=function(){ tries=0; lastRx=Date.now(); $("link").textContent="live";
    send("GET SCHEMA").then(function(ls){ SCHEMA=[]; ls.forEach(function(l){
      if(l.indexOf("K,")===0) SCHEMA.push(l.split(",").slice(1)); }); buildSettings(); });
    send("GET *").then(function(ls){ VALS={}; ls.forEach(function(l){
      var i=l.indexOf("="); if(i>0) VALS[l.slice(0,i)]=l.slice(i+1); });
      if(!SCHEMA.length) send("GET SCHEMA").then(function(l2){ SCHEMA=[]; l2.forEach(function(l){
        if(l.indexOf("K,")===0) SCHEMA.push(l.split(",").slice(1)); }); buildSettings(); });
      else buildSettings(); });
    syncClock();   // the page is the surface clock source: post it, then show what landed
    pump(); };
  sock.onclose=function(){
    busy=null; outq=[]; lastRx=0; $("link").textContent="offline";
    var t=Math.min(8000,400*Math.pow(2,tries++));
    $("link").textContent="reconnect in "+Math.round(t/1000)+"s";
    setTimeout(connect,t);
  };
  sock.onerror=function(){ if(sock) sock.close(); };
  sock.onmessage=function(ev){
    lastRx=Date.now();
    var frames=String(ev.data).split(/[\r\n]+/);
    for(var fi=0;fi<frames.length;fi++) if(frames[fi]) handleFrame(frames[fi]);
  };
  function handleFrame(s){
    if(s.charAt(0)==="#"){
      if(s.indexOf("#boot")===0){ var a=s.split(" "); bootMs=+a[1]||0;
        $("clock").textContent="uptime "+Math.round(bootMs/1000)+"s"; return; }
      if(s.indexOf("dropped")>0) toast("command too long","bad");
      finish(); return;
    }
    logline(s);
    // The answer collector: every line between a command and its '#' marker
    // belongs to the outstanding command. Without this push the collector was
    // always empty — Settings ("no schema yet"), the Dives list and the Cells
    // histograms all resolved zero rows; the dashboard only survived because
    // T/D/CELLS lines render through their own branches above. Callers filter
    // by prefix (K,, HIST,, L,, KEY=), so unsolicited event lines landing
    // mid-reply are harmless.
    if(busy) busy.lines.push(s);
    var f=s.split(",");
    if(f[0]==="T") { T=f; state(); }
    else if(f[0]==="D") { D=f; state(); }
    else if(f[0]==="CELLS") { CELLS=f; check(); }
    else if(f[0]==="PROOF") { PROOF=f; state(); }
    else if(f[0]==="PRE") { PRE=f; check(); }
    else if(f[0]==="CALAIR") { CALAIR=f; check(); }
    else if(f[0]==="FAULT") { FAULT=f; state(); }
    else if(f[0]==="WIFI") { WIFI=f; state(); }
    else if(f[0]==="E") {
      var id=+f[2], val=+f[3];
      toast(eventName(id,val),(id===1||id===10||id===11)?"bad":"ok"); }
    else if(f[0]==="ERR") { toast(s,"bad"); }
  }
}
var ALARMS=["LOW pO2","HIGH pO2","CELL DRIFT","SENSOR FAIL","BATTERY","SP CLAMP","STUCK OPEN",
            "CELL PROOF","CELL EXCLUDED","CELL REJOIN","HYPOXIA"];
function eventName(id,val){
  if(id===1) return "ALARM "+(ALARMS[val]||("bit"+val))+" RAISED";
  if(id===2) return "alarm "+(ALARMS[val]||("bit"+val))+" cleared";
  if(id===7){var ca=["no calibration yet","sampling","calibrated","refused: cells still warming up",
                     "aborted: a dive started","failed: cells never settled","failed: no cells fitted"];
             return "CAL AIR: "+(ca[val]||("code "+val));}
  var n={3:"mode",4:"setpoint",5:"cell excluded",6:"cell rejoin",7:"CAL AIR",8:"CAL O2",
         9:"cell proof PASS",10:"cell proof FAIL",11:"clamp ON",12:"clamp OFF",
         13:"pre-dive PASS",14:"RESET",15:"clock unset"};
  return (n[id]||("event "+id))+" ("+val+")";
}

// ---- rendering ------------------------------------------------------------
var FLAGNAMES=["LOW pO2","HIGH pO2","CELL DRIFT","SENSOR","BATTERY","SP CLAMP","SOLENOID","PROOF FAIL"];
function flagBits(){ if(!T) return 0; var m=/FLAGS0x([0-9a-fA-F]{2})/.exec(T[8]||""); return m?parseInt(m[1],16):0; }

function state(){
  if(!T){ $("ppo2").textContent="--.-"; return; }
  var voted=+T[2], sp=+T[6], mode=T[8]?T[7]:"STANDBY";
  var bits=flagBits();
  $("ppo2").textContent=voted.toFixed(2);
  $("sp").textContent=sp.toFixed(2);
  $("depth").textContent=(+T[1]).toFixed(1)+" m";
  // battery is the field appended after FLAGS: absent on an older build
  var vbat=(T.length>9)?+T[9]:0;
  $("bat").textContent=vbat>0?(vbat.toFixed(2)+" V"):"n/a";
  $("flags").innerHTML="";
  var modePill=document.createElement("span");
  modePill.className="pill on "+(mode==="AUTO"?"ok":mode==="MANUAL"?"warn":"");
  modePill.textContent=mode; $("flags").appendChild(modePill);
  for(var i=0;i<8;i++) if(bits&(1<<i)){
    var p=document.createElement("span");
    p.className="pill on "+(i<3||i===6||i===7?"bad":"warn"); p.textContent=FLAGNAMES[i];
    $("flags").appendChild(p);
  }
  var cl=$("cells"); cl.innerHTML="";
  for(var c=0;c<3;c++){
    var v=+T[3+c];
    var d=document.createElement("div"); d.className="cell";
    d.innerHTML='<span class="k">cell '+(c+1)+'</span><span class="bar"><i style="width:'+
      Math.max(0,Math.min(100,v/1.9*100))+'%"></i></span><span class="n">'+v.toFixed(3)+'</span>';
    cl.appendChild(d);
  }
  if(D){
    var ndl=+D[1], tot=+D[2], ce=+D[3], cns=D[D.length-1];
    $("cn").textContent=(ce>0?ce.toFixed(1)+" m / ":"")+ (ndl<0?"n/a":ndl>=480?">480":ndl)+" min";
    var stops=[]; for(var s=4;s<D.length-1;s++) if(D[s].indexOf("|")>0) stops.push(D[s].replace("|"," for "));
    $("deco").textContent=(tot>0?tot.toFixed(1)+" min":"none")+(stops.length?" &#8212; "+stops.join(", "):"");
    $("cns").textContent=(+cns).toFixed(0)+" %";
  }
  if(PROOF){
    var st=["INACTIVE","IDLE","ARMED","RUNNING","PASS","FAIL"][+PROOF[1]]||"?";
    $("proofline").textContent="cell proof: "+st+"  banned 0"+PROOF[3]+"  attempts "+PROOF[4];
  }
  if(FAULT){
    var duty=+FAULT[3];
    $("inj").textContent=(FAULT[1]==="1"?"STUCK OPEN ":"")+(FAULT[2]==="1"?"FUSE TRIPPED ":"")+
      "duty "+duty.toFixed(1)+"%";
  }
  // Radio policy (WIFI,on,held_by,idle_left_s,stations — §4). Two rules here:
  // words match the console's transition line, so the same state reads the
  // same way in a bench log and on the phone; and the idle countdown is shown
  // ONLY while no station is associated, because `stations>0` rewinds the
  // window every tick — advertising "sleeps in 10:00" to a connected reader
  // would be a timer that can never fire.
  if(WIFI){
    var on=+WIFI[1],hold=+WIFI[2],idle=+WIFI[3],st=+WIFI[4];
    var HOLD=["","diver's WIFI OFF","dive in progress","depth","idle: no station"];
    var txt;
    if(on){
      txt="up &#183; "+st+(st===1?" station":" stations");
      if(!st&&idle>0) txt="sleeps in "+Math.floor(idle/60)+":"+
        ("0"+(idle%60)).slice(-2)+" &#183; no station";
      else if(!st&&!(+((VALS["WIFI_IDLE_MIN"]||"0").split(".")[0])))
        txt+=" &#183; idle timer off";
    }
    else txt="down &#183; "+(HOLD[hold]||"reason "+hold);
    $("radio").innerHTML=txt;
  }
  var cls="clean";
  if(bits&((1<<0)|(1<<1)|(1<<6)|(1<<7))) cls="alarm"; else if(bits) cls="warn";
  document.body.className=cls;
  diveActive=(mode==="AUTO"||mode==="MANUAL");
  lockSettings();
}
var CALAIRW=["no calibration since power-up","sampling","calibrated",
             "refused: cells still warming up","aborted: a dive started",
             "failed: cells never settled inside the spread band","failed: no cells fitted"];
var PRESTEPW=["warm-up","air","solenoid","depth","battery"];
function check(){
  if(!PRE) { $("pre").innerHTML='<span class="hint">no check data yet</span>'; return; }
  var names=["warmup","air","solenoid","depth","battery"], txt=["pending","running","PASS","FAIL"];
  var out="";
  for(var i=0;i<5&&PRE.length>2+i;i++){
    var s=+PRE[1+i], kind=s===2?"ok":s===3?"bad":s===1?"warn":"";
    out+='<span class="pill'+(kind?" on "+kind:"")+'">'+names[i]+" "+txt[s]+"</span>";
  }
  if(PRE[6]==="1") out+='<span class="pill on ok">AUTO unlocked</span>';
  $("pre").innerHTML=out;
  if(CELLS){
    var st=["idle","baseline","pulsing","recover","done"][+CELLS[1]]||"?";
    $("cellline").textContent="cell test "+st+"  passed "+CELLS[2]+"  failed "+CELLS[3]+
      (CELLS[4]==="1"?"  (running)":"");
  }
  // The air calibration's own verdict, not the reply to the button: `CAL AIR`
  // only ever says the request was accepted. The numbers are the raw ADC
  // averages and the worst per-cell spread &#8212; what tells a dead cell from a
  // fouled one.
  if(CALAIR){
    var c=+CALAIR[1];
    var st=c===1?("sampling &#8212; "+CALAIR[3]+" of 12 samples"):
            (CALAIRW[c]||("code "+c));
    var kind=c===2?"ok":(c===0||c===1)?"":"bad";
    var det="";
    if(c===2) det=" raw "+CALAIR[5]+"/"+CALAIR[6]+"/"+CALAIR[7]+"  worst spread "+CALAIR[4]+
                  "  &#8212; "+CALAIR[2]+"s ago";
    else if(c===3) det=" &#8212; "+CALAIR[8]+"s left before it will run";
    else if(c!==0&&c!==1&&c!==2) det=" &#8212; "+CALAIR[2]+"s ago";
    $("calline").innerHTML='<span class="pill'+(kind?" on "+kind:"")+'">air cal: '+st+
                           '</span><span class="hint">'+det+'</span>';
  } else $("calline").innerHTML='<span class="hint">air calibration: no answer from the controller yet</span>';
}

// ---- settings -------------------------------------------------------------
function buildSettings(){
  var host=$("sets");
  if(!SCHEMA.length){ host.innerHTML='<div class="card"><span class="hint">no schema yet</span></div>'; return; }
  host.innerHTML="";
  var groups={};
  SCHEMA.forEach(function(d){
    var key=d[0], g=key.indexOf("SP")===0||key==="D1"||key==="D2"||key==="SP_HYST"?"Setpoints":
      key.indexOf("ALLO")===0||key==="CNS_ALARM"?"Alarms":
      key.indexOf("INJ")===0?"Injection":key.indexOf("CELLPROOF")===0?"Cell proof":
      key.indexOf("CELL_")===0?"Voting":key.indexOf("CAL")===0?"Calibration":
      key.indexOf("GF")===0||key.indexOf("ASC")===0||key==="WATER_DENSITY"?"Deco":"System";
    (groups[g]=groups[g]||[]).push(d);
  });
  Object.keys(groups).forEach(function(g){
    var fs=document.createElement("fieldset"), lg=document.createElement("legend");
    lg.textContent=g; fs.appendChild(lg);
    groups[g].forEach(function(d){ fs.appendChild(field(d)); });
    var c=document.createElement("div"); c.className="card"; c.appendChild(fs); host.appendChild(c);
  });
  lockSettings();
}
function field(d){
  var key=d[0], type=d[1], min=+d[2], max=+d[3], flags=+d[4], def=d[5];
  var w=document.createElement("div"); w.className="set"; w.dataset.key=key;
  w.dataset.lock=(flags&1)?"1":"";
  var lab=document.createElement("div"); lab.className="lab";
  lab.innerHTML=key+((flags&2)?' <span class="pill on warn">safety</span>':"")+
    '<div class="rng">'+(type==="S"?"length "+min+".."+max:min+" .. "+max)+" &#183; def "+def+"</div>";
  w.appendChild(lab);
  var inp=document.createElement("input");
  inp.id="f_"+key; inp.name=key;
  if(type==="B"){ inp.type="checkbox"; inp.checked=(VALS[key]==="1"); }
  else if(type==="S"){ inp.type="text"; inp.value=VALS[key]||""; inp.minLength=min; inp.maxLength=max; }
  else { inp.type="number"; inp.step=type==="F"?"0.01":"1"; inp.min=min; inp.max=max; inp.value=VALS[key]||def; }
  inp.addEventListener("change",function(){ save(key,inp,flags); });
  w.appendChild(inp);
  return w;
}
function save(key,inp,flags){
  var v=(inp.type==="checkbox")?(inp.checked?"1":"0"):inp.value;
  if(inp.type==="number"){ var n=+v;
    if(!isFinite(n)||n<+inp.min||n>+inp.max){ toast(key+": enter "+inp.min+" .. "+inp.max,"bad"); return; } }
  if(inp.minLength&&v.length<inp.minLength){ toast(key+": too short","bad"); return; }
  send("SET "+key+"="+v).then(function(ls){
    var err=ls.filter(function(l){return l.indexOf("ERR")===0});
    if(err.length){ toast(err[0],"bad"); inp.value=VALS[key]||""; return; }
    VALS[key]=v;
    toast(ls.filter(function(l){return l.indexOf("OK")===0}).join(" &#183; ")||"saved",
      (flags&2)?"bad":"ok");
    if(key.indexOf("AP_")===0) toast("AP name/password apply after the next radio restart","ok");
  });
}
function lockSettings(){
  document.querySelectorAll(".set").forEach(function(w){
    var i=w.querySelector("input"); if(!i) return;
    var locked=diveActive&&w.dataset.lock==="1";
    w.classList.toggle("locked",locked);
    if(locked) i.setAttribute("disabled",""); else i.removeAttribute("disabled");
  });
}

// ---- dives ----------------------------------------------------------------
function refreshDives(){
  send("LOG LIST").then(function(ls){
    var rows=ls.filter(function(l){return l.indexOf("L,")===0});
    if(!rows.length){ $("divelist").innerHTML='<span class="hint">no dives recorded</span>'; return; }
    // L,<slot>,<epoch>,<seq>: the firmware lists newest-written first, and the
    // sequence is what makes that claim true when the clock was never set. The
    // seq column is on the right because that is where new information goes.
    var h='<table><tr><th>#</th><th>start</th><th>seq</th><th></th></tr>';
    rows.forEach(function(l){
      var f=l.split(","), slot=+f[1], ep=+f[2], seq=f.length>3?+f[3]:0;
      var when=ep>100000?new Date(ep*1000).toLocaleString()
                        :"no clock at that start";
      h+="<tr><td>"+slot+"</td><td>"+when+"</td><td>"+(seq>0?seq:"&#8212;")+
         "</td><td><button class='act' data-slot='"+slot+
         "'>view</button> <a class='act' style='display:inline-block;padding:12px' href='/dive/"+
         slot+".csv'>csv</a></td></tr>";
    });
    $("divelist").innerHTML=h+"</table>";
    Array.prototype.forEach.call($("divelist").querySelectorAll("button"),function(b){
      b.addEventListener("click",function(){ showDive(+b.dataset.slot); }); });
  });
}
// ---- surface log: the machine's own calibration / pre-dive record ---------
// Fetched over HTTP, not the socket: /surface.csv is the log's only export path,
// the same rows the firmware's own CSV formatter produces. Rendered newest first,
// because that is the order a human reads it in, even though the file is stored
// oldest first so the record just made can never be the one that is lost.
function refreshSurface(){
  var host=$("surflog"); host.innerHTML="loading&#8230;";
  fetch("/surface.csv").then(function(r){ return r.text(); }).then(function(txt){
    var head=null, rows=[];
    txt.split("\n").forEach(function(l){
      var f=l.split(",");
      if(f[0]==="SURFACE"&&f.length>4) head=f;
      else if(f[0]==="R"&&f.length>12) rows.push(f);
    });
    if(!head){
      host.innerHTML='<span class="hint">the controller sent no SURFACE identity row '+
                     '&#8212; the log did not read</span>'; return;
    }
    if(!rows.length){
      host.innerHTML='<span class="hint">no surface records yet &#8212; nothing calibrated '+
                     'and no pre-dive check finished since this log began</span>'; return;
    }
    var n=+head[2], lo=+head[3], hi=+head[4], lost=(hi-lo+1)-n;
    // The identity row's own count and sequence range are compared with the rows
    // actually delivered: a file whose header says 256 records and streams 200 is
    // truncated, and saying so is better than showing a short list as the truth.
    var bad=(n!==rows.length)?" &#183; <b>header says "+n+" records, "+rows.length+
                              " arrived: file truncated</b>":"";
    var h='<div class="hint">'+rows.length+" record"+(rows.length===1?"":"s")+
          " &#183; seq "+lo+".."+hi+
          (lost>0?(" &#183; "+lost+" older record"+(lost===1?"":"s")+
                   " shed by capacity"):" &#183; none shed")+bad+"</div>"+
          '<table><tr><th>seq</th><th>when</th><th>what</th>'+
          '<th>result</th><th>numbers</th></tr>';
    rows.reverse().forEach(function(f){
      var seq=f[1], when=f[3], id=+f[4], name=f[5], val=+f[6];
      var raw=[f[7],f[8],f[9]], spread=f[10], samples=f[11], flags=+f[12];
      var what=name==="cal_air"?"air calibration":
                name==="pre_dive_pass"?"pre-dive checks":
                name==="pre_dive_fail"?"pre-dive checks":name;
      var res="", nums="", kind="";
      if(name==="cal_air"){
        res=CALAIRW[val]||("code "+val);
        kind=val===2?"ok":(val===0||val===1)?"":"bad";
        if(val===2) nums="raw "+raw.join("/")+" &#183; worst spread "+spread+
                        " &#183; "+samples+" samples";
      } else if(name==="pre_dive_fail"){
        res="failed at "+(PRESTEPW[val]||("step "+val)); kind="bad";
      } else if(name==="pre_dive_pass"){ res="passed"; kind="ok"; }
      else res="value "+val;
      // Same pill the checks view uses, so "calibrated" and "passed" look like the
      // thing they describe instead of being another row of black text.
      h+="<tr><td>"+seq+"</td><td>"+when+(flags&1?" <span class='hint'>(carried, unverified)</span>":"")+
         "</td><td>"+what+"</td><td><span class='pill"+(kind?" on "+kind:"")+"'>"+res+
         "</span></td><td class='hint'>"+nums+"</td></tr>";
    });
    host.innerHTML=h+"</table>";
  }).catch(function(){ host.innerHTML='<span class="hint">could not read /surface.csv</span>'; });
}

// ---- cell histograms (GET HIST, firmware rolling 10-minute windows) -----
function drawHist(f){
  var c=+f[1], ws=+f[2], nw=+f[3], bw=+f[4]/100, age=+f[5], n=+f[6];
  var host=$("h"+c); if(!host) return;
  var counts=f.slice(7), mx=1;
  counts.forEach(function(x){ if(+x>mx) mx=+x; });
  var when=n?("last "+(age/60).toFixed(1)+" min"):("no samples yet");
  var h='<div class="k">cell '+(c+1)+' &#183; '+when+' &#183; busiest bin '+mx+' smp</div><div class="hist">';
  for(var i=0;i<counts.length;i++){ var v=+counts[i];
    h+='<i class="'+(v?"":"z")+'" style="height:'+Math.round(Math.max(0,v)/mx*100)+
       '%" title="'+(i*bw).toFixed(2)+' .. '+((i+1)*bw).toFixed(2)+' bar: '+v+'"></i>'; }
  h+='</div><div class="rng">0 .. '+(counts.length*bw).toFixed(2)+' bar ppO2</div>';
  host.innerHTML=h;
  if(c===0) $("hline").textContent="samples per "+bw.toFixed(2)+" bar bin, rolling "+
    nw+" windows of "+ws+" s (firmware-side)";
}
function refreshHist(){
  send("GET HIST").then(function(ls){
    var got=ls.filter(function(l){return l.indexOf("HIST,")===0});
    if(!got.length) toast(ls.join(" &#183; ")||"no histogram","bad");
    got.forEach(function(l){ drawHist(l.split(",")); });
  });
}
function showDive(slot){
  var v=$("diveview"); v.hidden=false; v.innerHTML="loading dive "+slot+"&#8230;";
  fetch("/dive/"+slot+".csv").then(function(r){ return r.text(); }).then(function(txt){
    var snaps=[],meta=null,evts=[];
    txt.split("\n").forEach(function(l){
      var f=l.split(",");
      // v1 grammar (divelog.cpp snapshotCsvLine): f[1] t_s, f[2] depth_m,
      // f[3] voted, f[4-6] cells, f[7] SETPOINT, f[8] ALARMS(hex, the log
      // view: alarms.h), f[9] CNS. The indices were once sp:f[8] al:f[9] -
      // one column right, reading the alarm word as a setpoint and CNS as
      // an alarm. Nothing consumes these two fields today (the chart draws
      // t/d/p only), which is why the slip was harmless - and why the
      // grammar now sits here, where the next reader cannot repeat it.
      if(f[0]==="S"&&f.length>14) snaps.push({t:+f[1],d:+f[2],p:+f[3],sp:+f[7],al:+f[8]});
      if(f[0]==="M") meta=f;
      // The verdict rows. The chart says where the diver went; only these
      // say what the machine decided on the way: which alarm edges fired,
      // whether the cell proof failed, whether someone lifted the clamp and
      // how the calibration ended. The download has always carried them; a
      // surface reader could not see them at all (2026-11-26 queue item).
      if(f[0]==="E"&&f.length===4) evts.push(f);
    });
    if(!snaps.length){ v.innerHTML="dive "+slot+": no readable records"; return; }
    var h="<h3 style='margin-top:0'>dive "+slot+"</h3><canvas id='cv"+slot+"'></canvas>";
    if(meta) h+="<div class='hint'>duration "+Math.round(+meta[1]/60)+" min &#183; max "+
      (+meta[2]/100).toFixed(1)+" m &#183; ppO2 "+(+meta[3]/1000).toFixed(2)+" .. "+
      (+meta[4]/1000).toFixed(2)+" &#183; pulses "+meta[5]+" &#183; CNS peak "+meta[6]+"%</div>";
    // Verdicts in file order — which is occurrence order (the dive-file
    // grader pins the file's E, rows against the wire's sequence, so the
    // page may trust "after the failure came the lift" without sorting).
    // Words come from eventName, the same map the live toasts use: one
    // vocabulary for what the machine said, whether it said it now or in a
    // dive the diver reads back an hour later.
    h+="<div class='verdicts'><div class='hint'>"+(evts.length?
      "verdicts this dive: "+evts.length:
      "no verdict rows in this dive")+"</div>";
    for(var e=0;e<evts.length;e++){
      var f=evts[e], id=+f[2], val=+f[3], ts=+f[1];
      var kind=(id===1||id===10||id===11||id===7&&val>=5)?"bad":
               (id===2||id===6||id===12)?"ok":"warn";
      h+="<div class='vrow "+kind+"'>+"+Math.floor(ts/60)+":"+("0"+(ts%60)).slice(-2)+
        " &#183; "+eventName(id,val)+"</div>";
    }
    h+="</div>";
    v.innerHTML=h;
    drawChart($("cv"+slot),snaps);
  }).catch(function(){ v.innerHTML="could not read dive "+slot; });
}
function drawChart(cv,snaps){
  var w=cv.clientWidth||600,hh=cv.clientHeight||180;
  cv.width=w; cv.height=hh;
  var g=cv.getContext("2d"); g.clearRect(0,0,w,hh);
  var maxD=1,maxT=60;
  snaps.forEach(function(s){ if(s.d>maxD) maxD=s.d; if(s.t>maxT) maxT=s.t; });
  // depth (down) in amber, ppO2 scaled 0..2 in green
  g.strokeStyle="#ffb020"; g.beginPath();
  snaps.forEach(function(s,i){ var x=s.t/maxT*(w-4)+2, y=hh-(s.d/maxD)*(hh-8)-4;
    i?g.lineTo(x,y):g.moveTo(x,y); });
  g.stroke();
  g.strokeStyle="#39d98a"; g.beginPath();
  snaps.forEach(function(s,i){ var x=s.t/maxT*(w-4)+2, y=hh-(s.p/2)*(hh-8)-4;
    i?g.lineTo(x,y):g.moveTo(x,y); });
  g.stroke();
  g.fillStyle="#8c9bb0"; g.font="11px system-ui";
  g.fillText("depth "+maxD.toFixed(1)+" m",4,12); g.fillText("ppO2 2.0",w-56,12);
}

// ---- tabs + actions -------------------------------------------------------
Array.prototype.forEach.call(document.querySelectorAll("#tabs button"),function(b){
  b.addEventListener("click",function(){
    view=b.dataset.v;
    document.querySelectorAll("#tabs button").forEach(function(x){ x.classList.toggle("on",x===b); });
    document.querySelectorAll("main section").forEach(function(s){
      s.classList.toggle("on",s.id==="v-"+view); });
    if(view==="dives") refreshDives();
    if(view==="surface") refreshSurface();
    if(view==="hist") refreshHist();
    if(view==="check") { send("GET PRE"); send("GET CELLS"); send("GET CALAIR"); }
  });
});
Array.prototype.forEach.call(document.querySelectorAll("button[data-c]"),function(b){
  b.addEventListener("click",function(){
    var c=b.dataset.c;
    if(c==="DIVE END"&&!confirm("End the dive and close the log?")) return;
    if(c==="CELLPROOF OVERRIDE"&&!confirm("Lift the cell-proof clamp? The machine re-applies it if the cells still disagree.")) return;
    send(c).then(function(ls){ toast(ls.join(" &#183; "),ls.some(function(l){return l.indexOf("ERR")===0})?"bad":"ok"); });
  });
});
$("cmdform").addEventListener("submit",function(e){
  e.preventDefault();
  var v=$("cline").value.trim(); if(!v) return;
  $("cline").value=""; send(v);
});
$("refresh").addEventListener("click",refreshDives);
$("surfreq").addEventListener("click",refreshSurface);

// ---- cadence --------------------------------------------------------------
setInterval(function(){
  var stale=(Date.now()-lastRx>3000);
  if(stale){ document.body.classList.add("stale"); $("stale").hidden=false; }
  else { document.body.classList.remove("stale"); $("stale").hidden=true; }
},1000);
setInterval(function(){ if(sock&&sock.readyState===1) send("GET T"); },2000);
setInterval(function(){ if(sock&&sock.readyState===1&&(view==="status"||view==="check")) send("GET D"); },2400);
setInterval(function(){ if(sock&&sock.readyState===1&&view==="check"){ send("GET PRE"); send("GET CELLS"); send("GET PROOF"); send("GET FAULT"); send("GET CALAIR"); } },2400);
// The radio row: the page cannot show the policy, it can only ask for it —
// GET WIFI is the same line the handset and hw_accept's grade_wifi read.
setInterval(function(){ if(sock&&sock.readyState===1&&view==="status") send("GET WIFI"); },2400);
setInterval(function(){ if(sock&&sock.readyState===1&&view==="hist") refreshHist(); },4000);
connect();
</script>
</body></html>
)HTML";
// clang-format on
