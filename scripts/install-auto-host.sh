#!/usr/bin/env bash
# Installs the auto-host watcher as a systemd service.
# Run with sudo on the Pi after `cargo build --release` in host/.

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
unit="$here/tmc-new-era-host.service"

if [ "$(id -u)" -ne 0 ]; then
    echo "re-running with sudo"
    exec sudo "$0" "$@"
fi

chmod +x "$here/auto-host.sh"
install -m 644 "$unit" /etc/systemd/system/tmc-new-era-host.service
systemctl daemon-reload
systemctl enable --now tmc-new-era-host.service
systemctl --no-pager status tmc-new-era-host.service || true

echo
echo "logs: journalctl -u tmc-new-era-host -f"
