"""Live demo: phone camera -> tracker -> swarm planner -> UDP -> both bots. Logs every tick and every trial.

    python scripts/run_live.py                       # real camera and bots, from config.toml
    python scripts/run_live.py --dry-run             # synthetic camera + simulated bots, no hardware
    python scripts/run_live.py --recalibrate         # redo the 4-click arena calibration

Keys:  SPACE start/pause   x emergency stop (also pauses)   n new trial   y/f mark the finished trial a success/failure
       q quit
The bots do NOT move until you press SPACE. Their firmware also stops them 0.5 s after the last packet.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm import protocol  # noqa: E402
from swarm.io import LatestFrame, UdpLink  # noqa: E402
from swarm.config import arena_from, bots_from, load_config, swarm_params_from  # noqa: E402
from swarm.live import LiveSystem  # noqa: E402
from swarm.protocol import Command  # noqa: E402
from swarm.sim.world import SimParams, make_world  # noqa: E402
from swarm.swarm import Phase, Swarm  # noqa: E402
from swarm.vision.synth import SyntheticCamera  # noqa: E402
from swarm.vision.target import load_ranges  # noqa: E402
from swarm.vision.tracker import ArenaTracker, click_corners, draw_overlay  # noqa: E402

DT = 0.05  # 20 Hz


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config.toml"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--recalibrate", action="store_true")
    ap.add_argument("--target-color", default="red", help="red|orange|yellow|green|blue, or 'custom' (see scripts/pick_target_color.py)")
    ap.add_argument("--bots", default="", help="comma-separated bot IDs to use, e.g. 1 (default: every bot in config.toml)")
    ap.add_argument("--no-display", action="store_true", help="no window (SSH/tests); the run then cannot be started from the keyboard")
    ap.add_argument("--autostart", action="store_true",
                    help="start immediately. Refused unless every bot IP is loopback (127.0.0.1): for tests, never for real bots")
    ap.add_argument("--loop-video", action="store_true", help="loop a video FILE source (tests only)")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="exit after this many seconds (tests); 0 = run until quit")
    ap.add_argument("--timeout", type=float, default=240.0, help="give up on a trial after this many seconds")
    ap.add_argument("--cal", default=str(ROOT / "logs" / "calibration.json"))
    ap.add_argument("--seed", type=int, default=0, help="dry-run only")
    ap.add_argument("--auto", type=int, default=0, metavar="N",
                    help="dry-run only: run N trials with no window or keys, record outcomes from the simulator")
    a = ap.parse_args()

    cfg = load_config(a.config)
    arena, params = arena_from(cfg), swarm_params_from(cfg)
    bots = bots_from(cfg)
    if a.bots:
        wanted = {int(x) for x in a.bots.split(",")}
        missing = wanted - set(bots)
        if missing:
            sys.exit(f"--bots {a.bots}: no such bot in config.toml: {sorted(missing)}")
        bots = {i: v for i, v in bots.items() if i in wanted}
    ids = sorted(bots)
    tracker = ArenaTracker(arena)

    world = cam = link = stream = None
    if a.auto and not a.dry_run:
        sys.exit("--auto only makes sense with --dry-run (on hardware a human judges the outcome)")
    if a.dry_run:
        world = make_world(a.seed, len(ids), arena, SimParams(detect_radius=params.detect_radius))
        cam = SyntheticCamera(arena)
        tracker.calibrate_from_points([tuple(p) for p in cam.image_corners])
    else:
        try:
            stream = LatestFrame(cfg["camera"]["url"], loop=a.loop_video)
        except OSError as e:
            sys.exit(str(e))
        link = UdpLink(bots)
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

    (ROOT / "logs").mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    tick_log = open(ROOT / "logs" / f"live_{stamp}.csv", "w", newline="")
    tw = csv.writer(tick_log)
    tw.writerow(["t", "phase"] + [f"{k}{i}" for i in ids for k in ("x", "y", "th", "L", "R", "laser")])
    trials_path = ROOT / "logs" / "trials.csv"
    new_file = not trials_path.exists()
    trials = open(trials_path, "a", newline="")
    trw = csv.writer(trials)
    if new_file:
        trw.writerow(["timestamp", "trial", "mode", "t_found_s", "t_mark_s", "outcome"])

    color = a.target_color
    if color == "custom":
        color_file = ROOT / "logs" / "target_color.json"
        if not color_file.exists():
            sys.exit(f"{color_file} missing: run scripts/pick_target_color.py first")
        color = load_ranges(color_file)

    def fresh() -> LiveSystem:
        return LiveSystem(tracker, Swarm(arena, ids, params), color, params.detect_radius)

    if a.autostart and not a.dry_run and any(ip != "127.0.0.1" for ip, _ in bots.values()):
        sys.exit("--autostart is refused: it would start real bots moving without you pressing SPACE")
    live, trial, running, t0 = fresh(), 1, bool(a.auto) or a.autostart, time.time() if a.autostart else 0.0
    t_start = time.time()
    t_found = t_mark = None
    pending_outcome = False
    print("ready. Press SPACE in the video window to start; x = emergency stop; q = quit.")
    try:
        while True:
            t_loop = time.time()
            if world is not None:
                frame = cam.render({i: b.pose for i, b in world.bots.items()}, world.target, a.target_color if a.target_color != "custom" else "red")
            else:
                frame, ts = stream.read()
                if frame is None or time.time() - ts > 0.5:  # stale or missing video: never drive blind
                    link.send({i: Command() for i in ids})
                    time.sleep(0.05)
                    continue
            now = (world.t - t0) if (a.auto and running) else ((time.time() - t0) if running else 0.0)
            st = live.step(frame, now, run=running)
            cmds = st.cmds if running else {i: Command() for i in ids}
            if running:
                if st.detection is not None and t_found is None:
                    t_found = now
                if st.phase is Phase.MARK and t_mark is None:
                    t_mark = now
                if now > a.timeout and st.phase is not Phase.DONE:
                    running, pending_outcome = False, True
                    cmds = {i: Command() for i in ids}
                    print(f"trial {trial} TIMED OUT after {a.timeout:.0f}s in phase {st.phase.value}. Press f (failure).")
                if st.phase is Phase.DONE:
                    running, pending_outcome = False, True
                    cmds = {i: Command() for i in ids}
                    print(f"trial {trial} finished: found {t_found}, marked {t_mark}. Press y (success) or f (failure).")
                    if a.auto:
                        for _ in range(int(1.0 / DT)):  # let the final laser-off land before judging
                            for i in ids:
                                world.send(i, protocol.encode(i, 0, Command()))
                            world.step(DT)
                        trw.writerow([stamp, trial, "dry-run", t_found, t_mark, "success" if world.marked else "failure"])
                        trials.flush()
                        print(f"  auto result: {'success' if world.marked else 'failure'}")
                        if trial >= a.auto:
                            break
                        trial += 1
                        live, t_found, t_mark, pending_outcome = fresh(), None, None, False
                        world = make_world(a.seed + trial, len(ids), arena, SimParams(detect_radius=params.detect_radius))
                        running, t0 = True, world.t
            if a.auto and running and now > 300:  # safety net: a stuck trial counts as a failure
                trw.writerow([stamp, trial, "dry-run", t_found, t_mark, "failure(timeout)"])
                trials.flush()
                if trial >= a.auto:
                    break
                trial += 1
                live, t_found, t_mark = fresh(), None, None
                world = make_world(a.seed + trial, len(ids), arena, SimParams(detect_radius=params.detect_radius))
                t0 = world.t
            if world is not None:
                for i, c in cmds.items():
                    world.send(i, protocol.encode(i, 0, c))
                world.step(DT)
            else:
                link.send(cmds)
            tw.writerow([f"{now:.2f}", st.phase.value] + [v for i in ids for v in (
                (f"{st.poses[i].x:.3f}", f"{st.poses[i].y:.3f}", f"{st.poses[i].theta:.3f}") if i in st.poses else ("", "", ""))
                + (cmds[i].left, cmds[i].right, int(cmds[i].laser))])

            if a.auto:
                continue  # headless: no window, run as fast as the CPU allows
            if a.max_seconds and time.time() - t_start > a.max_seconds:
                break
            if a.no_display:
                time.sleep(max(0.0, DT - (time.time() - t_loop)))
                continue
            view = draw_overlay(frame, st.poses, tracker)
            if st.target_seen is not None:
                cv2.circle(view, tracker.world_to_pixel(st.target_seen), 14, (0, 255, 255), 2)
            status = "RUNNING" if running else ("PAUSED - SPACE to start" if not pending_outcome else "DONE - press y / f")
            lost = [i for i in ids if i not in st.poses]
            cv2.putText(view, f"trial {trial}  {st.phase.value.upper()}  {status}  t={now:.1f}s" + (f"  LOST {lost}" if lost else ""),
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("swarm (SPACE start/pause, x stop, n new, y/f result, q quit)", view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("x") or (key == ord(" ") and running):
                running = False
                if link:
                    link.stop_all()
                print("paused / emergency stop")
            elif key == ord(" ") and not pending_outcome:
                live = fresh()  # never reuse state from before the start
                t_found = t_mark = None
                running, t0 = True, time.time()
            elif key == ord("n"):
                running = False
                if link:
                    link.stop_all()
                live, t_found, t_mark, pending_outcome = fresh(), None, None, False
                if world is not None:
                    world = make_world(a.seed + trial, len(ids), arena, SimParams(detect_radius=params.detect_radius))
                print("new trial: reposition bots and target, then press SPACE")
            elif key in (ord("y"), ord("f")) and pending_outcome:
                trw.writerow([stamp, trial, "dry-run" if world else "hardware", t_found, t_mark,
                              "success" if key == ord("y") else "failure"])
                trials.flush()
                print(f"trial {trial} recorded as {'success' if key == ord('y') else 'failure'}")
                trial += 1
                live, t_found, t_mark, pending_outcome = fresh(), None, None, False
                if world is not None:
                    world = make_world(a.seed + trial, len(ids), arena, SimParams(detect_radius=params.detect_radius))
            time.sleep(max(0.0, DT - (time.time() - t_loop)))
    finally:
        if link:
            link.stop_all()
        if stream:
            stream.close()
        tick_log.close()
        trials.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
