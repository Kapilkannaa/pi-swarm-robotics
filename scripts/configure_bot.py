"""Create private per-bot Wi-Fi header with the recorded motor flags. No flashing."""
import argparse
import getpass
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def configure(bot_id, ssid, password, enable_motors=False):
    if bot_id not in (1, 2):
        raise ValueError("only the two documented robots have known direction flags")
    if not 8 <= len(password.encode()) <= 63:
        raise ValueError("WPA2 password must be 8..63 bytes")
    if not ssid or len(ssid.encode()) > 32:
        raise ValueError("SSID must be 1..32 bytes")
    text = f'''// PRIVATE, generated. Never include in the teammate package.
#pragma once
#define WIFI_SSID {json.dumps(ssid)}
#define WIFI_PASSWORD {json.dumps(password)}
#define BOT_ID {bot_id}
#define UDP_PORT 4210
#define SWAP_SIDES true
#define INVERT_LEFT false
#define INVERT_RIGHT {str(bot_id == 1).lower()}
#define MARKER_ONBOARD_LED true
#define MOTORS_ENABLED {str(enable_motors).lower()}
'''
    out = ROOT / f"firmware/bot/bot_config_bot{bot_id}.h"
    out.write_text(text)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bot", type=int, choices=[1, 2], required=True)
    ap.add_argument("--ssid", default="YOUR_SSID")
    ap.add_argument("--enable-motors", action="store_true")
    a = ap.parse_args()
    out = configure(a.bot, a.ssid, getpass.getpass("Hotspot password (hidden): "), a.enable_motors)
    print(f"Wrote private header for bot {a.bot}; motors_enabled={a.enable_motors}. Password not printed.")
