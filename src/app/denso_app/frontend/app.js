/* ============================================================
   PNK Motor Watch — browser side.

   The server sends raw numbers under the names ros2_control used, plus a
   timestamp per joint. Everything that turns that into meaning happens
   here: mapping interface names, working out staleness, deciding what is
   hot, choosing a colour, laying out panels. See config.js for the knobs
   and API.md for the wire format.

   Why this way round: the server is meant to be rewritten in C++, so the
   less it knows the smaller that job is. Judgement is also the part that
   changes most often, and changing it here costs a browser refresh
   instead of a rebuild and a restart on the robot.
   ============================================================ */

"use strict";

const CFG = window.DENSO_CONFIG;
const API = (window.DENSO_API || "").replace(/\/$/, "");

/* ---------- where a joint belongs ---------- */

const GROUP_OF = new Map();
CFG.groups.forEach(g => g.joints.forEach(j => GROUP_OF.set(j, g.key)));

const UNASSIGNED = { key: "unassigned", title: "Unassigned", note: "not listed in config.js" };

/* ---------- decoding a frame ---------- */

/* Server time when the last frame was built, and the browser instant it
   landed. Ages are measured against server time carried forward by the
   browser's own elapsed time — so when the link dies, ages keep growing
   instead of freezing at whatever the last frame said. */
let serverT = 0, rxAt = 0;

function nowServer() {
  return serverT + (performance.now() / 1000 - rxAt);
}

const motors = new Map();   // id -> decoded state

/* Every field a mapped interface can land in. Cleared before each frame is
   applied: a frame carries the joint's complete interface set, so merging
   into the previous one would keep showing a value the robot has stopped
   sending — exactly the kind of stale number this whole page exists to
   catch. */
const MAPPED_FIELDS = new Set(Object.values(CFG.interfaces));

function decode(frame) {
  serverT = frame.t;
  rxAt = performance.now() / 1000;

  for (const [id, raw] of Object.entries(frame.joints ?? {})) {
    const m = motors.get(id) ?? { id, group: GROUP_OF.get(id) ?? UNASSIGNED.key, extra: {} };
    m.stamp = raw.stamp ?? 0;
    for (const f of MAPPED_FIELDS) delete m[f];
    m.extra = {};
    for (const [iface, value] of Object.entries(raw)) {
      if (iface === "stamp") continue;
      const key = CFG.interfaces[iface];
      // An interface this page has never heard of is still a number the
      // robot published. Keep it — it shows up in the detail drawer.
      if (key) m[key] = value; else m.extra[iface] = value;
    }
    motors.set(id, m);
  }
}

function ageOf(m) {
  return m.stamp ? nowServer() - m.stamp : Infinity;
}

/* Order matters more than the thresholds. Silence outranks everything:
   position, velocity and effort keep their last value forever, so a
   reading from half a second ago is not evidence of anything. */
function severity(m) {
  if (ageOf(m) > CFG.staleAfterS) return "stale";
  if (m.status !== undefined) {
    const nib = Math.round(m.status) & 0x0F;
    if (nib === 0x0) return "off";
    if (nib >= CFG.faultNibbleMin) return "crit";
  }
  if (m.trotor !== undefined) {
    if (m.trotor >= CFG.tempCritC) return "crit";
    if (m.trotor >= CFG.tempWarnC) return "warn";
  }
  return "ok";
}

function statusName(v) {
  if (v === undefined) return null;
  return CFG.statusNames[Math.round(v) & 0x0F] ?? "UNKNOWN";
}

function pillText(m) {
  const age = ageOf(m);
  if (age > CFG.staleAfterS) {
    return age === Infinity ? "NO DATA"
      : "STALE " + Math.min(99999, age * 1000).toFixed(0) + "ms";
  }
  return statusName(m.status) ?? "REPORTING";
}

/* ---------- DOM ---------- */

const rows = new Map();

function shortName(id) {
  return id.replace(/^openarm_(left|right)_/, "").replace(/^(left|right)_/, "")
           .replace(/^head_joint_/, "").replace(/_joint$/, "");
}

function esc(s) {
  return String(s ?? "").replace(/[&<>"']/g,
    c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

function buildPanels() {
  const armsRow = document.getElementById("rowArms");
  const lowerRow = document.getElementById("rowLower");
  CFG.groups.forEach((g, i) => {
    const sec = document.createElement("section");
    sec.className = "panel";
    sec.innerHTML = `<header><h2>${esc(g.title)}</h2><span class="note">${esc(g.note ?? "")}</span></header>
      <div class="motors" id="grp-${esc(g.key)}"></div>`;
    (i < 2 ? armsRow : lowerRow).appendChild(sec);
    g.joints.forEach(id => {
      motors.set(id, { id, group: g.key, extra: {} });
      buildRow(id, g.key);
    });
  });
}

function panelHost(group) {
  let host = document.getElementById("grp-" + group);
  if (host) return host;
  const sec = document.createElement("section");
  sec.className = "panel";
  sec.innerHTML = `<header><h2>${esc(UNASSIGNED.title)}</h2>
    <span class="note">${esc(UNASSIGNED.note)}</span></header>
    <div class="motors" id="grp-${esc(group)}"></div>`;
  document.getElementById("rowLower").appendChild(sec);
  return document.getElementById("grp-" + group);
}

function buildRow(id, group) {
  const el = document.createElement("button");
  el.type = "button";
  el.className = "m no-temp s-stale";
  el.innerHTML = `
    <span class="stripe"></span>
    <span class="name">${esc(shortName(id))}</span>
    <span class="can"></span>
    <span>
      <span class="posbar"><span class="zero"></span><span class="fill"></span></span>
      <span class="posval"></span>
    </span>
    <span class="pill">NO DATA</span>`;
  el.addEventListener("click", () => openDrawer(id));
  panelHost(group).appendChild(el);
  rows.set(id, {
    el,
    pos:  el.querySelector(".posbar .fill"),
    posv: el.querySelector(".posval"),
    pill: el.querySelector(".pill"),
    therm: null
  });
  return rows.get(id);
}

/* Thermal columns appear the moment the robot starts reporting them. Until
   openarm_hardware exports temperature_mos / temperature_rotor there is
   nothing to draw, and an empty gauge would read as a measurement of zero
   rather than as a gap. */
function ensureThermal(r) {
  if (r.therm) return r.therm;
  const span = document.createElement("span");
  span.className = "therm";
  span.innerHTML = `
    <span class="t"><span class="lbl">MOS</span><span class="track"><span class="fill"></span></span><span class="deg"></span></span>
    <span class="t"><span class="lbl">ROT</span><span class="track"><span class="fill"></span></span><span class="deg"></span></span>`;
  r.el.insertBefore(span, r.pill);
  r.el.classList.remove("no-temp");
  r.therm = span;
  return span;
}

/* ---------- render ---------- */

function tempClass(c) {
  return c >= CFG.tempCritC ? "crit" : c >= CFG.tempWarnC ? "warn" : "";
}

function render() {
  let reporting = 0, hottest = null, faults = 0, stale = 0, peakEff = 0;

  for (const m of motors.values()) {
    const r = rows.get(m.id) ?? buildRow(m.id, m.group);
    const sev = severity(m);
    const age = ageOf(m);

    r.el.className = "m" + (r.therm ? "" : " no-temp") + " s-" + sev;

    const pos = m.pos ?? 0;
    const frac = Math.max(-1, Math.min(1, pos / Math.PI));
    r.pos.style.left  = (frac < 0 ? 50 + frac * 50 : 50) + "%";
    r.pos.style.width = Math.abs(frac) * 50 + "%";
    r.posv.textContent = (pos >= 0 ? " " : "") + pos.toFixed(3) + " rad";

    if (m.tmos !== undefined || m.trotor !== undefined) {
      const therm = ensureThermal(r);
      const bars = therm.querySelectorAll(".track .fill");
      const degs = therm.querySelectorAll(".deg");
      [m.tmos, m.trotor].forEach((v, i) => {
        if (v === undefined) { degs[i].textContent = "—"; return; }
        bars[i].style.width = Math.min(100, (v / CFG.tempScaleC) * 100) + "%";
        bars[i].className = "fill " + tempClass(v);
        degs[i].textContent = Math.round(v) + "°";
      });
      therm.classList.toggle("dim", sev === "stale" || sev === "off");
      if (m.trotor !== undefined) hottest = Math.max(hottest ?? 0, m.trotor);
    }

    r.pill.textContent = pillText(m);
    r.pill.className = "pill " + (sev === "ok" ? "" : sev);

    if (age <= CFG.staleAfterS) reporting++; else stale++;
    if (sev === "crit" && m.status !== undefined) faults++;
    peakEff = Math.max(peakEff, Math.abs(m.eff ?? 0));
  }

  set("mReport", `${reporting}<span class="u"> / ${motors.size}</span>`);
  set("mHot", hottest === null
    ? `<span class="u" style="font-size:13px">not exported yet</span>`
    : `${Math.round(hottest)}<span class="u"> °C</span>`);
  set("mFault", faults);
  set("mStale", stale);
  set("mEffort", `${peakEff.toFixed(1)}<span class="u"> N·m</span>`);

  flag("mHotBox", hottest === null ? ""
    : hottest >= CFG.tempCritC ? "is-crit" : hottest >= CFG.tempWarnC ? "is-warn" : "");
  flag("mFaultBox", faults > 0 ? "is-crit" : "");
  flag("mStaleBox", stale > 0 ? "is-stale" : "");

  const up = (Date.now() - t0) / 1000;
  document.getElementById("uptime").textContent =
    String(Math.floor(up / 60)).padStart(2, "0") + ":" + String(Math.floor(up % 60)).padStart(2, "0");

  renderPower();
  refreshDrawer();
}

function set(id, html) { document.getElementById(id).innerHTML = html; }
function flag(id, cls) { document.getElementById(id).className = "metric " + cls; }

/* ---------- fault feed ---------- */

let logLines = [];

function renderFeed() {
  const host = document.getElementById("feed");
  if (!logLines.length) {
    host.innerHTML = `<div class="empty">Nothing at WARN or above since this page connected.</div>`;
    return;
  }
  host.innerHTML = logLines.slice(0, 40).map(l => {
    const lv = CFG.logLevels[l.level] ?? "warn";
    const d = new Date((l.stamp ?? 0) * 1000);
    const ts = d.toTimeString().slice(0, 8) + "." +
               String(d.getMilliseconds()).padStart(3, "0");
    return `<div class="f lv-${lv}">
      <span class="stripe"></span>
      <span class="ts">${esc(ts)}</span>
      <span class="src">${esc(l.name)}</span>
      <span class="msg">${esc(l.msg)}</span>
      <span class="code">${esc(lv === "crit" ? "ERROR" : "WARN")}</span>
    </div>`;
  }).join("");
}

/* ---------- the socket ---------- */

const t0 = Date.now();
let ws = null, retry = 500, linkUp = false;

function wsUrl() {
  if (API) return API.replace(/^http/, "ws") + "/ws/motors";
  return `${location.protocol === "https:" ? "wss:" : "ws:"}//${location.host}/ws/motors`;
}

function connect() {
  ws = new WebSocket(wsUrl());
  ws.onopen = () => { linkUp = true; retry = 500; setLink(true); };
  ws.onmessage = e => {
    const frame = JSON.parse(e.data);
    decode(frame);
    if (frame.log) { logLines = frame.log; renderFeed(); }
    const hz = frame.rates?.dynamic_joint_states;
    if (hz !== undefined) {
      document.getElementById("busRate0").textContent = hz.toFixed(0) + " Hz";
    }
    render();
  };
  ws.onclose = () => {
    linkUp = false;
    setLink(false, "websocket closed");
    setTimeout(connect, retry);          // back off, up to 5 s
    retry = Math.min(retry * 2, 5000);
  };
  ws.onerror = () => ws.close();
}

function setLink(up, detail) {
  const badge = document.getElementById("feedBadge");
  badge.innerHTML = `<span class="dot ${up ? "" : "crit"}"></span>${up ? "Live" : "No link"}`;
  badge.style.background = up ? "var(--ok-bg)" : "var(--crit-bg)";
  badge.style.color = up ? "var(--ok)" : "var(--crit)";
  badge.style.borderColor = "transparent";
  document.getElementById("linkBanner").hidden = up;
  if (!up && detail) document.getElementById("linkDetail").textContent = detail;
  document.getElementById("busDot0").className = "dot" + (up ? "" : " crit");
}

/* ---------- arm power ---------- */

const arming = {};
const lastCall = {};

function buildArmControls() {
  const host = document.getElementById("power");
  CFG.arms.forEach(a => {
    const box = document.createElement("div");
    box.className = "arm-ctl";
    box.id = "ctl-" + a.key;
    box.innerHTML = `
      <span class="who"><span class="n">${esc(a.title)}</span><span class="d">${esc(a.note ?? "")}</span></span>
      <span class="acts">
        <button class="btn stop" type="button" data-side="${esc(a.side)}" data-key="${esc(a.key)}" data-act="disable">Disable</button>
        <button class="btn go"   type="button" data-side="${esc(a.side)}" data-key="${esc(a.key)}" data-act="enable">Enable</button>
      </span>
      <span class="arm-state"><span class="dot"></span><span class="txt">—</span><span class="verb"></span></span>`;
    host.appendChild(box);
  });
  host.querySelectorAll(".btn").forEach(btn => btn.addEventListener("click", onArmClick));
}

function onArmClick(ev) {
  const btn = ev.currentTarget;
  const { side, key, act } = btn.dataset;
  if (act === "disable") { runCommand(side, key, "disable"); return; }

  // Enabling puts torque back on seven joints, so it takes a second
  // deliberate press. Disabling — the safe direction — never does.
  if (arming[key]) {
    clearTimeout(arming[key]); arming[key] = null;
    runCommand(side, key, "enable");
    return;
  }
  btn.textContent = "Confirm enable";
  btn.classList.add("arming");
  arming[key] = setTimeout(() => {
    arming[key] = null;
    btn.textContent = "Enable";
    btn.classList.remove("arming");
  }, 4000);
}

async function runCommand(side, key, act) {
  const btns = document.querySelectorAll(`#ctl-${key} .btn`);
  btns.forEach(b => b.disabled = true);

  let status = 0, detail = "";
  try {
    const r = await fetch(`${API}/arm/${side}/${act}`, { method: "POST" });
    status = r.status;
    detail = (await r.json()).detail ?? "";
  } catch {
    detail = "could not reach the server";
  }

  lastCall[key] = `POST /arm/${side}/${act} → ${status || "no reply"} · ${new Date().toTimeString().slice(0, 8)}`;
  if (status !== 200) {
    logLines.unshift({
      stamp: Date.now() / 1000, level: 40,
      name: `${side} arm · HTTP ${status || "unreachable"}`, msg: detail
    });
    renderFeed();
  }
  btns.forEach(b => b.disabled = false);
  renderPower();
}

function renderPower() {
  for (const a of CFG.arms) {
    const box = document.getElementById("ctl-" + a.key);
    if (!box) continue;
    const mine = [...motors.values()].filter(m => m.group === a.key);
    const sevs = mine.map(severity);
    const faulted = sevs.includes("crit");
    const quiet = mine.length > 0 && sevs.every(s => s === "stale");
    const off = mine.length > 0 && sevs.every(s => s === "off" || s === "stale") && !quiet;

    const en = box.querySelector('[data-act="enable"]');
    if (!arming[a.key]) { en.textContent = "Enable"; en.classList.remove("arming"); }

    box.querySelector(".arm-state .dot").className =
      "dot" + (faulted ? " crit" : (quiet || off) ? " stale" : "");
    box.querySelector(".arm-state .txt").textContent =
      !linkUp ? "no telemetry"
      : faulted ? "FAULT ON ONE JOINT"
      : quiet ? "not reporting"
      : off ? `DISABLED · ${mine.length} joints released`
      : `REPORTING · ${mine.length} joints`;
    box.querySelector(".arm-state .verb").textContent =
      lastCall[a.key] ? "— " + lastCall[a.key] : "";
  }
}

/* ---------- detail drawer ---------- */

let drawerId = null;

function drawerValues(m) {
  const hex = v => "0x" + (Math.round(v) & 0x0F).toString(16).toUpperCase();
  const age = ageOf(m);
  const num = (v, d, unit) => v === undefined ? "not exported" : v.toFixed(d) + unit;
  return `
    <dt>Status</dt><dd>${m.status === undefined ? "not exported" : hex(m.status) + " " + statusName(m.status)}</dd>
    <dt>Latched fault</dt><dd>${m.fault === undefined ? "—" : hex(m.fault) + " " + statusName(m.fault)}</dd>
    <dt>Frame age</dt><dd>${age === Infinity ? "never seen" : (age * 1000).toFixed(0) + " ms"}</dd>
    <dt>Position</dt><dd>${num(m.pos, 4, " rad")}</dd>
    <dt>Velocity</dt><dd>${num(m.vel, 3, " rad/s")}</dd>
    <dt>Effort</dt><dd>${num(m.eff, 2, " N·m")}</dd>
    <dt>MOS temp</dt><dd>${m.tmos === undefined ? "not exported" : Math.round(m.tmos) + " °C"}</dd>
    <dt>Coil temp</dt><dd>${m.trotor === undefined ? "not exported" : Math.round(m.trotor) + " °C"}</dd>
    <dt>Group</dt><dd>${esc(m.group)}</dd>
    ${Object.entries(m.extra ?? {}).map(([k, v]) =>
      `<dt>${esc(k)}</dt><dd>${Number(v).toFixed(3)}</dd>`).join("")}`;
}

function openDrawer(id) {
  closeDrawer();
  drawerId = id;
  const d = document.createElement("aside");
  d.className = "drawer";
  d.id = "drawer";
  d.innerHTML = `<h3>${esc(id)}</h3><dl class="kv" id="drawerKv"></dl>
    <button class="ghost close" type="button">Close</button>`;
  d.querySelector(".close").addEventListener("click", closeDrawer);
  document.body.appendChild(d);
  refreshDrawer();
  d.querySelector(".close").focus();
}

function closeDrawer() {
  document.getElementById("drawer")?.remove();
  drawerId = null;
}

function refreshDrawer() {
  if (!drawerId) return;
  const kv = document.getElementById("drawerKv");
  const m = motors.get(drawerId);
  if (kv && m) kv.innerHTML = drawerValues(m);
}

document.addEventListener("keydown", e => { if (e.key === "Escape") closeDrawer(); });

/* ---------- theme ---------- */

document.getElementById("themeBtn").addEventListener("click", () => {
  const cur = document.documentElement.getAttribute("data-theme");
  const dark = cur ? cur === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
  document.documentElement.setAttribute("data-theme", dark ? "light" : "dark");
});

/* ---------- go ---------- */

buildPanels();
buildArmControls();
renderFeed();
render();
setLink(false, "connecting");
connect();

/* Keep ages honest between frames — and while the link is down. */
setInterval(render, 250);
