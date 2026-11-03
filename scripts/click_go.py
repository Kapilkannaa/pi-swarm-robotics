"""Click on the camera picture and the bot drives there. The bot is found by its red power LED (no printed markers).

    python scripts/click_go.py --bot 2                  # real camera + real bot (config.toml)
    python scripts/click_go.py --dry-run                # simulated bot and camera, nothing real moves
    python scripts/click_go.py --bot 2 --recalibrate    # redo the 4-click floor calibration

Mouse: LEFT click = go there.  RIGHT click = stop.      Keys: x stop, m show what the detector sees, [ / ] LED brightness
threshold -/+, r recalibrate, q quit.   The bot only moves toward a clicked goal; the firmware also stops it 0.5 s after the last packet.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm import protocol  # noqa: E402
from swarm.clickgo import ClickGo, ClickParams  # noqa: E402
from swarm.marker_go import MarkerGo
from swarm.offload_client import BackgroundOffload
from swarm.config import arena_from, bots_from, load_config  # noqa: E402
from swarm.geometry import Pose  # noqa: E402
from swarm.io import LatestFrame, UdpLink  # noqa: E402
from swarm.protocol import Command  # noqa: E402
from swarm.sim.firmware_model import FirmwareModel  # noqa: E402
from swarm.sim.world import SimBot, SimParams, make_world  # noqa: E402
from swarm.vision.led import led_mask  # noqa: E402
from swarm.vision.synth import SyntheticCamera  # noqa: E402
from swarm.vision.tracker import ArenaTracker, click_corners  # noqa: E402

DT = 0.05


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config.toml"))
    ap.add_argument("--bot", type=int, default=0, help="bot ID (default: the first in config.toml)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--recalibrate", action="store_true")
    ap.add_argument("--cal", default=str(ROOT / "logs" / "calibration.json"))
    ap.add_argument("--led-offset", type=float, default=0.05, help="how far BEHIND the bot's centre the red LED sits (m)")
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--goals", default="", help='scripted goals in metres, e.g. "0.8,0.2;0.2,0.6" (for headless runs/tests)')
    ap.add_argument("--max-seconds", type=float, default=0.0)
    ap.add_argument("--loop-video", action="store_true", help="loop a video FILE source (tests only)")
    ap.add_argument("--source", help="phone video URL or laptop camera index; overrides config")
    ap.add_argument("--tracking", choices=("led", "aruco"), default="led")
    ap.add_argument("--marker-id", type=int, help="printed marker ID, independent of firmware bot ID")
    ap.add_argument("--offload", help="optional Pi background worker URL, e.g. http://PI_IP:8770")
    a = ap.parse_args()

    cfg = load_config(a.config)
    arena, bots = arena_from(cfg), bots_from(cfg)
    bot_id = a.bot or sorted(bots)[0]
    marker_id = a.marker_id if a.marker_id is not None else bot_id
    if bot_id not in bots:
        sys.exit(f"bot {bot_id} is not in config.toml (has {sorted(bots)})")
    c = cfg.get("control", {})
    params = ClickParams(min_cmd=float(c.get("min_cmd", 35)), latency_s=float(c.get("latency_s", 0.4)), led_offset=a.led_offset)
    tracker = ArenaTracker(arena)
    scripted = [tuple(float(v) for v in g.split(",")) for g in a.goals.split(";") if g.strip()]
    if scripted and not a.dry_run and bots[bot_id][0] != "127.0.0.1":
        sys.exit("--goals is refused for a real bot: it would start driving without you clicking. Click on the picture instead.")

    world = cam = link = stream = None
    if a.dry_run:
        cam = SyntheticCamera(arena, px_per_m=600, margin_px=90)  # smaller than the demo camera: faster, LED still ~5 px
        tracker.calibrate_from_points([tuple(p) for p in cam.image_corners])
        world = make_world(1, 1, arena, SimParams(camera_latency_s=0.3, packet_loss=0.03))
        world.bots = {bot_id: SimBot(bot_id, Pose(0.2, 0.2, 1.0), 1.0, 0.92, FirmwareModel(bot_id))}
        params.latency_s = max(params.latency_s, 0.4)
    else:
        try:
            stream = LatestFrame(a.source or cfg["camera"]["url"], loop=a.loop_video)
        except OSError as e:
            sys.exit(str(e))
        link = UdpLink({bot_id: bots[bot_id]})
        cal = Path(a.cal)
        if cal.exists() and not a.recalibrate:
            tracker.load(cal)
        else:
            print("waiting for a camera frame...")
            while True:
                f, _ = stream.read()
                if f is not None:
                    break
                time.sleep(0.1)
            tracker.calibrate_from_points(click_corners(f))
            cal.parent.mkdir(exist_ok=True)
            tracker.save(cal)
            print(f"calibration saved to {cal}")

    cg = MarkerGo(tracker, arena, marker_id, params) if a.tracking == "aruco" else ClickGo(tracker, arena, params)
    offload = None
    if a.offload:
        token_file = ROOT / "local_token.txt"
        token = token_file.read_text().strip() if token_file.exists() else ""
        offload = BackgroundOffload(a.offload, token)
    submitted_goal, logged_job = None, None
    (ROOT / "logs").mkdir(exist_ok=True)
    log_f = open(ROOT / "logs" / f"clickgo_{time.strftime('%Y%m%d_%H%M%S')}.csv", "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["t", "status", "led_x", "led_y", "cx", "cy", "theta", "goal_x", "goal_y", "L", "R"])
    clicks: list = []
    trail: deque = deque(maxlen=250)
    show_mask, t_start, seq, g_idx, hold_until = False, time.time(), 0, 0, 0.0
    win = "click-to-go: LEFT click = go, RIGHT click = stop, x = stop, q = quit"

    def on_mouse(event: int, x: int, y: int, *_: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append(("go", (float(x), float(y))))
        elif event == cv2.EVENT_RBUTTONDOWN:
            clicks.append(("stop", None))

    if not a.no_display:
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, on_mouse)
    if scripted:
        cg.goal = scripted[0]
    print("ready. Click on the picture to send the bot there." if not scripted else f"scripted goals: {scripted}")
    status, last_t = "idle", time.time()
    try:
        while True:
            t_loop = time.time()
            dt = min(0.2, t_loop - last_t) if not a.dry_run else DT
            last_t = t_loop
            if world is not None:
                frame = (cam.render({marker_id: world.bots[bot_id].pose}) if a.tracking == "aruco" else
                         cam.render_leds({i: b.pose for i, b in world.bots.items()}, offset=a.led_offset))
            else:
                frame, ts = stream.read()
                if frame is None or time.time() - ts > 0.5:  # missing or stale video: never drive blind
                    if a.max_seconds and time.time() - t_start > a.max_seconds:
                        break
                    if not a.no_display and cv2.waitKey(1) & 0xFF in (ord("x"), ord("q")):
                        cg.stop()
                        if link: link.stop_all()
                        break
                    link.send({bot_id: Command()})
                    time.sleep(0.05)
                    continue
            while clicks:
                kind, px = clicks.pop(0)
                if kind == "go":
                    g = cg.set_goal_pixel(px)
                    print(f"goal ({g[0]:.2f}, {g[1]:.2f}) m")
                else:
                    cg.stop()
                    if link:
                        link.stop_all()
            st = cg.step(frame, dt)
            if offload:
                if submitted_goal is not None and st.goal != submitted_goal:
                    offload.cancel()
                    submitted_goal = None
                if st.goal is None:
                    if submitted_goal is not None: offload.cancel()
                    submitted_goal = None
                elif st.goal != submitted_goal and st.centre is not None and st.theta is not None:
                    offload.submit({"arena": [arena.width, arena.height],
                        "pose": [*st.centre, st.theta], "goal": list(st.goal),
                        "control": {"min_cmd": params.min_cmd, "latency_s": params.latency_s}, "trials": 12})
                    submitted_goal = st.goal
                snapshot = offload.snapshot()
                if snapshot["result"] and snapshot["job_id"] != logged_job:
                    with open(ROOT / "logs/offload_jobs.jsonl", "a", encoding="utf-8") as jobs:
                        jobs.write(__import__("json").dumps(snapshot["result"]) + "\n")
                    logged_job = snapshot["job_id"]
            status = st.status
            if world is not None:
                seq += 1
                world.send(bot_id, protocol.encode(bot_id, seq, st.cmd))
                world.step(DT)
            else:
                link.send({bot_id: st.cmd})
            if st.led_px:
                trail.append(st.led_px)
            tt = time.time() - t_start
            log.writerow([f"{tt:.2f}", st.status] + [f"{v:.3f}" if v is not None else "" for v in
                          (*(st.led_world or (None, None)), *(st.centre or (None, None)), st.theta,
                           *(st.goal or (None, None)))] + [st.cmd.left, st.cmd.right])

            if scripted and status == "arrived":
                if hold_until == 0.0:
                    hold_until = time.time() + 1.0
                elif time.time() > hold_until:
                    g_idx, hold_until = g_idx + 1, 0.0
                    if g_idx >= len(scripted):
                        break
                    cg.goal = scripted[g_idx]
            if a.max_seconds and tt > a.max_seconds:
                break

            if not a.no_display:
                view = frame.copy()
                for pt in trail:
                    cv2.circle(view, (int(pt[0]), int(pt[1])), 1, (255, 160, 0), -1)
                if st.led_px:
                    cv2.circle(view, (int(st.led_px[0]), int(st.led_px[1])), 12, (0, 255, 0), 2)
                if st.centre and st.theta is not None:
                    c0 = tracker.world_to_pixel(st.centre)
                    tip = tracker.world_to_pixel((st.centre[0] + 0.12 * math.cos(st.theta), st.centre[1] + 0.12 * math.sin(st.theta)))
                    cv2.arrowedLine(view, c0, tip, (0, 0, 255), 3)
                if st.goal:
                    g_px = tracker.world_to_pixel(st.goal)
                    cv2.drawMarker(view, g_px, (0, 255, 255), cv2.MARKER_CROSS, 28, 3)
                    cv2.circle(view, g_px, int(0.04 * abs(tracker.world_to_pixel((arena.width, 0))[0] - tracker.world_to_pixel((0, 0))[0]) / arena.width) + 2, (0, 255, 255), 2)
                d = cg.driver
                hud = f"{status.upper()}  heading {'?' if st.theta is None else f'{math.degrees(st.theta):.0f} deg'}  speed {getattr(d, 'speed', params.speed0):.2f} m/s  turn {getattr(d, 'omega', params.omega0):.1f} rad/s"
                cv2.putText(view, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                if st.led_px is None:
                    cv2.putText(view, "ROBOT NOT FOUND" + (" (m mask, [ ] thresholds)" if a.tracking == "led" else f" (marker {marker_id})"), (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                if offload:
                    snap = offload.snapshot()
                    text = "Pi offload: " + snap["state"]
                    if snap["result"]:
                        rr = snap["result"]
                        text = f"Pi {rr.get('host', '?')}: {rr['work_steps']} simulated steps | advice: {rr['recommendation']}"
                    cv2.putText(view, text, (10, view.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 180, 40), 1)
                cv2.imshow(win, view)
                if show_mask:
                    cv2.imshow("what the detector sees", led_mask(frame, cg.led, cg._roi))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("x"):
                    cg.stop()
                    if link:
                        link.stop_all()
                elif key == ord("m") and a.tracking == "led":
                    show_mask = not show_mask
                    if not show_mask:
                        cv2.destroyWindow("what the detector sees")
                elif key == ord("[") and a.tracking == "led":
                    cg.led.v_min = max(60, cg.led.v_min - 10)
                elif key == ord("]") and a.tracking == "led":
                    cg.led.v_min = min(250, cg.led.v_min + 10)
                elif key == ord("r") and stream is not None:
                    tracker.calibrate_from_points(click_corners(frame))
                    tracker.save(a.cal)
                    cg._roi = None
            time.sleep(max(0.0, DT - (time.time() - t_loop)))
    finally:
        if offload: offload.close()
        if link:
            link.stop_all()
        if stream:
            stream.close()
        log_f.close()
        cv2.destroyAllWindows()
    if world is not None:
        p = world.bots[bot_id].pose
        print(f"final true position ({p.x:.2f}, {p.y:.2f})" + (f"; last goal {cg.goal}, error {math.hypot(p.x - cg.goal[0], p.y - cg.goal[1]) * 1000:.0f} mm" if cg.goal else ""))


if __name__ == "__main__":
    main()
