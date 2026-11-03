#!/usr/bin/env bash
# Flash one bot with its own settings: scripts/flash.sh 1 [port]
# Copies firmware/bot/bot_config_bot<N>.h to bot_config.h first, so the two bots' IDs and
# motor flags can never be mixed up by editing a shared file.
set -euo pipefail
n="${1:?usage: flash.sh <bot number> [port]}"
port="${2:-/dev/ttyUSB0}"
cd "$(dirname "$0")/.."
src="firmware/bot/bot_config_bot${n}.h"
[ -f "$src" ] || { echo "missing $src"; exit 1; }
cp "$src" firmware/bot/bot_config.h
grep -E "BOT_ID|SWAP_SIDES|INVERT_" firmware/bot/bot_config.h
arduino-cli compile --upload -p "$port" --fqbn esp8266:esp8266:d1_mini firmware/bot
