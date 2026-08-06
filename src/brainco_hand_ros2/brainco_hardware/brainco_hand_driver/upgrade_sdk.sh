#!/usr/bin/env bash
#
# Vendor a BrainCo Stark SDK release into this package's vendor/dist/.
#
# The package links against its own private copy of the SDK under vendor/dist/,
# which is independent of any standalone SDK checkout elsewhere on the machine.
# This script replaces that copy, keeping a timestamped backup, and reports
# which SDK symbols the driver source still depends on so an API break is
# visible BEFORE the next colcon build rather than as a link error.
#
# It only moves files. It never edits driver source — porting call sites stays
# a deliberate, reviewable change.
#
# Usage:
#   ./upgrade_sdk.sh --check              # report only, touch nothing
#   ./upgrade_sdk.sh                      # vendor from the default source tree
#   ./upgrade_sdk.sh --source /path/to/brainco-hand-sdk
#   ./upgrade_sdk.sh --restore            # roll back to the newest backup

set -euo pipefail

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
readonly PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly VENDOR_DIR="${PKG_DIR}/vendor"
readonly DIST_DIR="${VENDOR_DIR}/dist"
readonly VERSION_FILE="${VENDOR_DIR}/VERSION"
readonly BACKUP_ROOT="${VENDOR_DIR}/backup"

readonly DEFAULT_SOURCE_DIR="${HOME}/work/brainco-hand-sdk"
readonly SDK_LIB_NAME="libbc_stark_sdk.so"
readonly SDK_HEADER_NAME="stark-sdk.h"

# Functions the driver's data path calls. If any of these disappears, the port
# is no longer mechanical and the upgrade must be reconsidered.
readonly REQUIRED_SYMBOLS=(
  stark_get_motor_status
  stark_set_finger_positions_and_durations
  stark_get_device_info
  stark_get_finger_unit_mode
  stark_set_finger_unit_mode
  modbus_open
  modbus_close
  auto_detect_modbus_revo2
  set_can_tx_callback
  set_can_rx_callback
)

# Functions the driver's open/close plumbing calls that v2.0.2 removed. Present
# in the source SDK means no port needed; absent means these call sites break.
readonly FRAGILE_SYMBOLS=(
  init_cfg
  canfd_init
  free_device_handler
)

readonly COLOR_RED='\033[0;31m'
readonly COLOR_GREEN='\033[0;32m'
readonly COLOR_YELLOW='\033[0;33m'
readonly COLOR_NONE='\033[0m'

log_info() { printf "%b\n" "  $*"; }
log_ok() { printf "%b\n" "  ${COLOR_GREEN}OK${COLOR_NONE}    $*"; }
log_warn() { printf "%b\n" "  ${COLOR_YELLOW}WARN${COLOR_NONE}  $*"; }
log_error() { printf "%b\n" "  ${COLOR_RED}ERROR${COLOR_NONE} $*" >&2; }
log_section() { printf "\n%b\n" "${COLOR_YELLOW}== $* ==${COLOR_NONE}"; }

die() {
  log_error "$*"
  exit 1
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
source_dir="${DEFAULT_SOURCE_DIR}"
check_only=0
do_restore=0

while [[ $# -gt 0 ]]; do
  case "$1" in
  --source)
    [[ $# -ge 2 ]] || die "--source needs a directory argument"
    source_dir="$2"
    shift 2
    ;;
  --check)
    check_only=1
    shift
    ;;
  --restore)
    do_restore=1
    shift
    ;;
  -h | --help)
    sed -n '3,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    exit 0
    ;;
  *)
    die "unknown argument '$1' (try --help)"
    ;;
  esac
done

# ---------------------------------------------------------------------------
# Restore mode
# ---------------------------------------------------------------------------
if [[ ${do_restore} -eq 1 ]]; then
  log_section "Restore"
  [[ -d ${BACKUP_ROOT} ]] || die "no backup directory at ${BACKUP_ROOT}"

  newest_backup="$(find "${BACKUP_ROOT}" -mindepth 1 -maxdepth 1 -type d | sort | tail -1)"
  [[ -n ${newest_backup} ]] || die "no backups found under ${BACKUP_ROOT}"

  log_info "restoring from ${newest_backup}"
  rm -rf "${DIST_DIR}"
  cp -a "${newest_backup}/dist" "${DIST_DIR}"
  if [[ -f "${newest_backup}/VERSION" ]]; then
    cp -a "${newest_backup}/VERSION" "${VERSION_FILE}"
  fi
  log_ok "restored; rebuild with 'colcon build --packages-select brainco_hand_driver'"
  exit 0
fi

# ---------------------------------------------------------------------------
# Validate the source tree
# ---------------------------------------------------------------------------
log_section "Source"

[[ -d ${source_dir} ]] || die "source tree not found: ${source_dir}"

src_lib="${source_dir}/dist/shared/linux/${SDK_LIB_NAME}"
src_header="${source_dir}/dist/include/${SDK_HEADER_NAME}"

[[ -f ${src_lib} ]] || die "missing ${src_lib} (run '${source_dir}/download-lib.sh' first)"
[[ -f ${src_header} ]] || die "missing ${src_header}"

src_version="unknown"
if [[ -f "${source_dir}/VERSION" ]]; then
  src_version="$(head -1 "${source_dir}/VERSION" | tr -d '\r')"
fi

current_version="none"
if [[ -f ${VERSION_FILE} ]]; then
  current_version="$(head -1 "${VERSION_FILE}" | tr -d '\r')"
fi

log_info "source:  ${source_dir}"
log_info "version: ${src_version}"
log_info "vendored now: ${current_version}"

# ---------------------------------------------------------------------------
# Architecture check — a mismatched .so links but never loads at runtime
# ---------------------------------------------------------------------------
log_section "Architecture"

host_arch="$(uname -m)"
src_arch="$(file -b "${src_lib}" | grep -oE 'x86-64|aarch64|ARM aarch64' | head -1 || true)"
[[ -n ${src_arch} ]] || src_arch="unrecognised"

log_info "host:       ${host_arch}"
log_info "source .so: ${src_arch}"

arch_matches=0
case "${host_arch}:${src_arch}" in
x86_64:x86-64) arch_matches=1 ;;
aarch64:aarch64 | aarch64:"ARM aarch64") arch_matches=1 ;;
esac

if [[ ${arch_matches} -eq 1 ]]; then
  log_ok "architecture matches"
else
  log_warn "architecture MISMATCH — this .so will not load on ${host_arch}"
  log_warn "re-run '${source_dir}/download-lib.sh' on the target machine;"
  log_warn "it selects linux-arm64.zip automatically when uname -m is aarch64"
fi

# ---------------------------------------------------------------------------
# Symbol audit — decides whether the port is mechanical or not
# ---------------------------------------------------------------------------
log_section "Symbol audit"

command -v nm >/dev/null 2>&1 || die "'nm' not found; install binutils"

exported_symbols="$(nm -D --defined-only "${src_lib}" 2>/dev/null | awk '$2 == "T" { print $3 }')"

missing_required=0
for symbol in "${REQUIRED_SYMBOLS[@]}"; do
  if grep -qx "${symbol}" <<<"${exported_symbols}"; then
    log_ok "${symbol}"
  else
    log_error "${symbol} — MISSING, the driver data path needs this"
    missing_required=$((missing_required + 1))
  fi
done

needs_port=0
for symbol in "${FRAGILE_SYMBOLS[@]}"; do
  if grep -qx "${symbol}" <<<"${exported_symbols}"; then
    log_ok "${symbol} (still present, no port needed)"
  else
    log_warn "${symbol} — removed, call sites must be ported"
    needs_port=$((needs_port + 1))
  fi
done

if [[ ${missing_required} -gt 0 ]]; then
  die "${missing_required} required symbol(s) missing — refusing to vendor this SDK"
fi

if [[ ${needs_port} -gt 0 ]]; then
  log_section "Port required after vendoring"
  cat <<'PORT_NOTES'
  This SDK removes symbols the open/close plumbing uses. Affected call sites:

    src/brainco_hand_api.cpp
      init_cfg(protocol_type, level)  ->  init_logging(level)
        (protocol type moved into the handle; the local becomes unused)

    src/canfd_session.cpp, src/socketcan_session.cpp
      canfd_init(master_id)
        ->  init_device_handler_can(STARK_PROTOCOL_TYPE_CAN_FD,
                                    master_id, arb_baud_bps, data_baud_bps)
      free_device_handler  ->  close_device_handler
        (also in the matching .hpp unique_ptr deleter typedefs)

  The data path (stark_get_motor_status / stark_set_finger_positions_and_durations)
  is unchanged, so reading state and opening/closing the hand need no edits.
PORT_NOTES
fi

if [[ ${check_only} -eq 1 ]]; then
  log_section "Check only"
  log_info "nothing was modified"
  exit 0
fi

# ---------------------------------------------------------------------------
# Back up the current vendored SDK
# ---------------------------------------------------------------------------
log_section "Backup"

if [[ -d ${DIST_DIR} ]]; then
  backup_dir="${BACKUP_ROOT}/${current_version}-$(date +%Y%m%d-%H%M%S)"
  mkdir -p "${backup_dir}"
  cp -a "${DIST_DIR}" "${backup_dir}/dist"
  if [[ -f ${VERSION_FILE} ]]; then
    cp -a "${VERSION_FILE}" "${backup_dir}/VERSION"
  fi
  log_ok "saved to ${backup_dir}"
  log_info "roll back with: $(basename "${BASH_SOURCE[0]}") --restore"
else
  log_warn "no existing ${DIST_DIR}, nothing to back up"
fi

# ---------------------------------------------------------------------------
# Vendor the new SDK
# ---------------------------------------------------------------------------
log_section "Vendor"

mkdir -p "${DIST_DIR}/include" "${DIST_DIR}/shared/linux"

install -m 0644 "${src_header}" "${DIST_DIR}/include/${SDK_HEADER_NAME}"
log_ok "include/${SDK_HEADER_NAME}"

install -m 0755 "${src_lib}" "${DIST_DIR}/shared/linux/${SDK_LIB_NAME}"
log_ok "shared/linux/${SDK_LIB_NAME}"

# The ZLG headers ship alongside the SDK and are needed by canfd_session.cpp.
# They are unrelated to the SocketCAN path, so a source tree without them is
# not an error.
src_zlg_dir="${source_dir}/dist/include/zlgcan"
if [[ -d ${src_zlg_dir} ]]; then
  mkdir -p "${DIST_DIR}/include/zlgcan"
  cp -a "${src_zlg_dir}/." "${DIST_DIR}/include/zlgcan/"
  log_ok "include/zlgcan/ (ZLG CAN FD headers)"
else
  log_warn "no zlgcan headers in the source tree; ENABLE_CANFD builds will fail"
fi

printf '%s\n' "${src_version}" >"${VERSION_FILE}"
log_ok "VERSION -> ${src_version}"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
log_section "Done"
log_info "vendored ${current_version} -> ${src_version}"

if [[ ${needs_port} -gt 0 ]]; then
  log_warn "port the call sites listed above BEFORE building, or the link will fail"
else
  log_info "no source changes needed"
fi

log_info "then: colcon build --packages-select brainco_hand_driver"
