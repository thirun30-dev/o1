#!/usr/bin/env bash
# Create a self-signed TLS certificate so rosbridge can serve wss://.
#
# Needed when the dashboard is opened over https:// (e.g. the Vercel
# deployment): browsers refuse plain ws:// connections from a secure page.
#
# Usage (on the Raspberry Pi):
#   ./make_rosbridge_cert.sh            # uses the Pi's current IP addresses
#   ./make_rosbridge_cert.sh 10.0.0.42  # add extra IPs / hostnames
#
# Then start the robot with:
#   ros2 launch optimus_bot bringup.launch.py rosbridge_ssl:=true
set -euo pipefail

OUT_DIR="${HOME}/.optimus_certs"
mkdir -p "${OUT_DIR}"
chmod 700 "${OUT_DIR}"

HOST="$(hostname)"
SAN="DNS:localhost,DNS:${HOST},DNS:${HOST}.local,IP:127.0.0.1"
for ip in $(hostname -I); do
  case "$ip" in *:*) continue ;; esac   # skip IPv6
  SAN="${SAN},IP:${ip}"
done
for extra in "$@"; do
  if [[ "$extra" =~ ^[0-9.]+$ ]]; then SAN="${SAN},IP:${extra}"; else SAN="${SAN},DNS:${extra}"; fi
done

openssl req -x509 -newkey rsa:2048 -nodes -days 825 \
  -keyout "${OUT_DIR}/rosbridge.key" \
  -out "${OUT_DIR}/rosbridge.crt" \
  -subj "/CN=${HOST}" \
  -addext "subjectAltName=${SAN}"

chmod 600 "${OUT_DIR}/rosbridge.key"
echo
echo "Certificate written to ${OUT_DIR} for: ${SAN}"
echo "If the Pi's IP changes, run this script again."
