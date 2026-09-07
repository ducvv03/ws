/* ============================================================
   Everything the page needs to make sense of a raw telemetry frame.

   The server sends numbers under the names ros2_control used, and a
   timestamp. It decides nothing. All of the meaning is here — which
   means changing a threshold, renaming a panel or teaching the page a
   new interface is an edit to this file and a browser refresh, with no
   rebuild and no restart of anything on the robot.
   ============================================================ */

/* Where the API lives.
   null  -> same origin this page was served from (the normal case).
   a URL -> point at a server elsewhere, e.g. while developing the page
            against a robot on the bench, or once the C++ server runs on
            its own port:  window.DENSO_API = "http://192.168.1.50:8080";
   A cross-origin server must send CORS headers — see API.md. */
window.DENSO_API = null;

window.DENSO_CONFIG = {

  /* Panels, in the order they appear. `joints` is what goes in each; a
     joint the server sends that is listed nowhere still shows up, in an
     "Unassigned" panel, rather than being silently dropped. */
  groups: [
    { key: "left_arm",  title: "Left arm",  note: "Damiao · can1", joints: [
      "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
      "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
      "openarm_left_joint7" ] },
    { key: "right_arm", title: "Right arm", note: "Damiao · can0", joints: [
      "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
      "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
      "openarm_right_joint7" ] },
    { key: "head",      title: "Head",      note: "Damiao · can2 / can3", joints: [
      "head_joint_vertical", "head_joint_horizontal" ] },
    { key: "left_hand", title: "Left hand", note: "Revo2 · Modbus — no thermal", joints: [
      "left_thumb_metacarpal_joint", "left_thumb_proximal_joint",
      "left_index_proximal_joint", "left_middle_proximal_joint",
      "left_ring_proximal_joint", "left_pinky_proximal_joint" ] },
    { key: "right_hand", title: "Right hand", note: "Revo2 · Modbus — no thermal", joints: [
      "right_thumb_metacarpal_joint", "right_thumb_proximal_joint",
      "right_index_proximal_joint", "right_middle_proximal_joint",
      "right_ring_proximal_joint", "right_pinky_proximal_joint" ] }
  ],

  /* Which arm each Arm-power button drives. Only the label side —
     the server has its own copy of the controller names. */
  arms: [
    { key: "left_arm",  side: "left",  title: "Left arm",  note: "7 joints · can1" },
    { key: "right_arm", side: "right", title: "Right arm", note: "7 joints · can0" }
  ],

  /* ros2_control interface name -> what this page calls it.
     Left column is the string a hardware plugin passes to StateInterface().
     Anything not listed still reaches the detail drawer, under its own name. */
  interfaces: {
    position:          "pos",
    velocity:          "vel",
    effort:            "eff",
    temperature_mos:   "tmos",
    temperature_rotor: "trotor",
    temperature_coil:  "trotor",
    status:            "status",
    fault:             "fault"
  },

  /* A joint whose last frame is older than this stopped answering.
     Matches DEFAULT_STATE_TIMEOUT_S in dm_motor_constants.hpp — keep the
     two in step. */
  staleAfterS: 0.1,

  /* Coil temperature, °C. */
  tempWarnC: 65,
  tempCritC: 80,
  tempScaleC: 95,     // full-scale for the little thermal bars

  /* Damiao status nibbles, from dm_motor_constants.hpp. Anything not in
     this table is UNKNOWN — never "healthy". */
  statusNames: {
    0x0: "DISABLED", 0x1: "ENABLED",  0x8: "OVERVOLTAGE", 0x9: "UNDERVOLTAGE",
    0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "COIL_OVERTEMP",
    0xD: "COMM_LOST", 0xE: "OVERLOAD", 0xF: "UNKNOWN"
  },
  faultNibbleMin: 0x8,

  /* rcl log levels -> feed severity. */
  logLevels: { 30: "warn", 40: "crit", 50: "crit" }
};
