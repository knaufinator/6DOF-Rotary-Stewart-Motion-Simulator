#!/usr/bin/env bash
# install.sh — one-command installer for the HIL bridge on the Voron.
# This IS the "create service" action from HIL_BRIDGE.md.
#
# Run ON the Voron (knaufinator@192.168.1.168), from a checkout of this repo:
#   cd 6DOF-Rotary-Stewart-Motion-Simulator/bridge
#   sudo ./install.sh
#
# It: copies bridge/ to /opt/hil-bridge, builds a venv, installs deps, installs
# + enables the systemd unit, and starts the service. Idempotent (re-runnable).
#
# Env overrides:
#   DEST=/opt/hil-bridge  SERVICE_USER=knaufinator  SERIAL_DEV=/dev/serial/by-id/...
set -euo pipefail

DEST="${DEST:-/opt/hil-bridge}"
SERVICE_USER="${SERVICE_USER:-knaufinator}"
SERIAL_DEV="${SERIAL_DEV:-}"        # empty => MOCK mode until you set it
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (sudo ./install.sh)." >&2
  exit 1
fi

echo "==> Installing HIL bridge to ${DEST} (user: ${SERVICE_USER})"

# 1. Copy source (exclude venv + caches).
mkdir -p "${DEST}"
for f in cobs.py serial_link.py udp_relay.py control_api.py bridge.py \
         requirements.txt test_client.py selftest.py PROTOCOL.md README.md; do
  install -m 0644 "${SRC_DIR}/${f}" "${DEST}/${f}"
done
chown -R "${SERVICE_USER}:${SERVICE_USER}" "${DEST}"

# 2. venv + deps.
echo "==> Creating venv + installing dependencies"
sudo -u "${SERVICE_USER}" python3 -m venv "${DEST}/.venv"
sudo -u "${SERVICE_USER}" "${DEST}/.venv/bin/pip" install --upgrade pip
sudo -u "${SERVICE_USER}" "${DEST}/.venv/bin/pip" install -r "${DEST}/requirements.txt"

# 3. Ensure the service user can reach the USB serial device.
if getent group dialout >/dev/null; then
  usermod -aG dialout "${SERVICE_USER}" || true
fi

# 4. Install the systemd unit, patching the serial device if provided.
echo "==> Installing systemd unit"
UNIT=/etc/systemd/system/hil_bridge.service
install -m 0644 "${SRC_DIR}/hil_bridge.service" "${UNIT}"
sed -i "s#^WorkingDirectory=.*#WorkingDirectory=${DEST}#" "${UNIT}"
sed -i "s#^User=.*#User=${SERVICE_USER}#" "${UNIT}"
sed -i "s#^ExecStart=.*#ExecStart=${DEST}/.venv/bin/python ${DEST}/bridge.py#" "${UNIT}"
if [[ -n "${SERIAL_DEV}" ]]; then
  sed -i "s#^Environment=HIL_SERIAL_DEV=.*#Environment=HIL_SERIAL_DEV=${SERIAL_DEV}#" "${UNIT}"
fi

# 5. Enable + (re)start.
systemctl daemon-reload
systemctl enable hil_bridge.service
systemctl restart hil_bridge.service

echo "==> Done. Status:"
systemctl --no-pager --full status hil_bridge.service || true
echo
echo "Logs:   journalctl -u hil_bridge.service -f"
echo "WS:     ws://<voron-ip>:8788   UDP motion: <voron-ip>:8767"
if [[ -z "${SERIAL_DEV}" ]]; then
  echo "NOTE: running in MOCK serial mode (HIL_SERIAL_DEV unset). Set it in ${UNIT} and 'systemctl restart hil_bridge' once the mini is attached."
fi
