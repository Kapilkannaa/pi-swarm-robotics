#!/bin/sh
# Install a compute-only Pi role. This service never sends robot UDP commands.
set -eu
cd "$(dirname "$0")/.."
root=$(pwd)
[ -f local_token.txt ] || python3 -c 'import secrets; open("local_token.txt","w").write(secrets.token_urlsafe(32))'
python3 - <<'PY'
from pathlib import Path
Path('pi.env').write_text('SWARM_TOKEN=' + Path('local_token.txt').read_text().strip() + '\n')
PY
chmod 600 local_token.txt pi.env
# Preview cleanup can restart the old coordinator, so stop it first.
systemctl --user stop pi-swarm-demo.service 2>/dev/null || true
systemctl --user disable --now pi-swarm.service 2>/dev/null || true
mkdir -p "$HOME/.config/systemd/user"
cat > "$HOME/.config/systemd/user/pi-offload.service" <<EOF
[Unit]
Description=Robotics background compute worker (no motor transport)

[Service]
WorkingDirectory=$root
EnvironmentFile=$root/pi.env
ExecStart=/usr/bin/python3 -m swarm.offload_server --port 8770
Restart=on-failure
RestartSec=2
NoNewPrivileges=true
UMask=0077

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable --now pi-offload.service
echo 'Compute-only worker enabled. Laptop controls robots directly. No video passes through Pi.'
