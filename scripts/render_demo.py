"""Render a software-only demonstration using the real tracker, controller and worker.

Optional video dependency: pip install imageio-ffmpeg
No camera device, robot socket or physical Raspberry Pi is used.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import secrets
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from swarm.clickgo import ClickParams
from swarm.geometry import Pose, dist
from swarm.marker_go import MarkerGo
from swarm.offload_client import BackgroundOffload
from swarm.offload_server import Worker
from swarm.planner import Arena
from swarm.protocol import encode
from swarm.sim.firmware_model import FirmwareModel
from swarm.sim.world import SimBot, SimParams, World
from swarm.vision.synth import SyntheticCamera
from swarm.vision.tracker import ArenaTracker

W, H, FPS = 1280, 800, 20
NAVY, TEAL, BG = '#203e65', '#087f8c', '#edf2f7'


def font(size, bold=False):
    for name in ([f'C:/Windows/Fonts/arial{"bd" if bold else ""}.ttf',
                  f'/usr/share/fonts/truetype/dejavu/DejaVuSans{"-Bold" if bold else ""}.ttf']):
        if Path(name).exists(): return ImageFont.truetype(name, size)
    return ImageFont.load_default(size=size)


def base(subtitle):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, W, 112), fill=NAVY)
    d.text((36, 21), 'PI SWARM', fill='white', font=font(32, True))
    d.text((36, 66), subtitle, fill='#cde6ee', font=font(20))
    return im, d


def card(title, lines):
    im, d = base('Vision-guided navigation with background edge computing')
    d.text((64, 164), title, fill=NAVY, font=font(35, True))
    d.text((64, 215), 'PI SWARM  /  PROJECT DEMONSTRATION', fill=TEAL, font=font(16, True))
    y = 270
    for heading, body in lines:
        d.rounded_rectangle((60, y, 1220, y+112), radius=12, fill='white')
        d.rectangle((60, y+15, 65, y+97), fill=TEAL)
        d.text((82, y+17), heading, fill=TEAL, font=font(22, True))
        d.text((82, y+55), body, fill=NAVY, font=font(20))
        y += 133
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, default=ROOT/'media')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    import imageio_ffmpeg
    encoder = imageio_ffmpeg.write_frames(str(a.out/'pi-swarm-simulation.mp4'),
        (W,H), fps=FPS, codec='libx264', pix_fmt_in='rgb24', pix_fmt_out='yuv420p',
        quality=7, macro_block_size=16, ffmpeg_log_level='error')
    encoder.send(None)
    frames_written, previews = 0, []

    def emit(im, count=1, preview=False):
        nonlocal frames_written
        arr = np.asarray(im)
        for _ in range(count): encoder.send(arr); frames_written += 1
        if preview: previews.append(im.resize((640, 400)))

    # Brief animated invitation, followed immediately by movement.
    for i in range(25):
        intro, d = base('Vision-guided navigation with background edge computing')
        d.text((64, 255), "Let's look at the simulation", fill=NAVY, font=font(47, True))
        d.text((66, 335), 'Follow the robot from its marker to the selected destination.', fill=TEAL, font=font(25))
        d.rounded_rectangle((66, 410, 1214, 418), radius=4, fill='#d4e0e8')
        d.rounded_rectangle((66, 410, 66+int(1148*(i+1)/25), 418), radius=4, fill=TEAL)
        emit(intro)

    arena = Arena(1, .75)
    cam = SyntheticCamera(arena, px_per_m=600, margin_px=60)
    tracker = ArenaTracker(arena)
    tracker.calibrate_from_points([tuple(p) for p in cam.image_corners])
    driver = MarkerGo(tracker, arena, 1, ClickParams(latency_s=.4))
    goal = (.75, .50)
    world = World(arena, goal, {2: SimBot(2, Pose(.2,.2,1), 1, .92, FirmwareModel(2))},
        SimParams(camera_latency_s=.3, packet_loss=.03), random.Random(7))
    token = secrets.token_urlsafe(32)
    server = Worker(('127.0.0.1', 0), token)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval':.05}, daemon=True)
    thread.start()
    client = BackgroundOffload(f'http://127.0.0.1:{server.server_port}', token)
    trace, path, clicked, saw_lost, zero_lost, result = [], [], False, False, True, None
    reached, seq = False, 0
    screenshot_saved = False
    try:
        for step in range(2400):
            observed = world.observe()[2]
            occluded = 5 <= world.t < 6
            frame = cam.render({} if occluded else {1: observed})
            if not clicked and world.t >= 1:
                driver.set_goal_pixel(tracker.world_to_pixel(goal))
                client.submit({'arena':[1,.75], 'pose':[observed.x, observed.y, observed.theta],
                    'goal':list(goal), 'trials':12})
                clicked = True
            state = driver.step(frame, .05)
            if occluded:
                saw_lost = True
                zero_lost &= state.cmd.left == 0 and state.cmd.right == 0
            world.send(2, encode(2, seq, state.cmd)); seq += 1
            world.step(.05)
            p = world.bots[2].pose
            path.append(tracker.world_to_pixel((p.x,p.y)))
            snap = client.snapshot()
            if snap['result'] is not None: result = snap['result']
            trace.append({'sim_time_s':round(world.t,3), 'true_x_m':p.x,'true_y_m':p.y,
                'true_theta_rad':p.theta,'status':state.status,
                'left':state.cmd.left,'right':state.cmd.right,'marker_occluded':occluded,
                'worker_state':snap['state']})
            if step % 2 == 0:
                im, d = base('Marker tracking / printed marker 1 / firmware bot 2')
                camera = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
                cd = ImageDraw.Draw(camera)
                corners = [tuple(map(int, p)) for p in cam.image_corners]
                cd.line(corners+[corners[0]], fill='#899aaa', width=2)
                if len(path)>1: cd.line(path, fill=TEAL, width=2)
                if clicked:
                    gx,gy = tracker.world_to_pixel(goal)
                    cd.ellipse((gx-11,gy-11,gx+11,gy+11),outline='#d75049',width=3)
                    cd.line((gx-17,gy,gx+17,gy),fill='#d75049',width=2)
                    cd.line((gx,gy-17,gx,gy+17),fill='#d75049',width=2)
                    cd.text((gx+17,gy-15),'selected goal',font=font(15),fill='#a82b29')
                im.paste(camera,(28,137))
                d.rounded_rectangle((774,137,1250,711),radius=12,fill='white')
                d.text((797,158),'LOCAL CONTROL',font=font(22,True),fill=TEAL)
                rows = [f'Simulated time: {world.t:5.1f} s', 'Playback: 2x simulated time',
                    f'State: {state.status}', f'Wheel commands: {state.cmd.left:+.0f}, {state.cmd.right:+.0f}',
                    f'Goal: ({goal[0]:.2f}, {goal[1]:.2f}) m',
                    f'True goal error: {1000*dist((p.x,p.y),goal):.0f} mm']
                for i,line in enumerate(rows): d.text((797,200+i*29),line,font=font(19),fill=NAVY)
                d.line((797,393,1227,393),fill='#dae2eb',width=2)
                d.text((797,411),'BACKGROUND COMPUTE',font=font(22,True),fill=TEAL)
                d.text((797,451),snap['state'],font=font(19),fill=NAVY)
                if result:
                    for i,row in enumerate(result['profiles']):
                        d.text((797,488+i*29),f"{row['profile']}: {row['successes']}/{row['trials']}, {row['mean_time_s']} s",font=font(18),fill=NAVY)
                    d.text((797,594),'Advice only. Controller unchanged.',font=font(19,True),fill=TEAL)
                d.text((797,647),'HTTP worker is running on this laptop.',font=font(17),fill='#526477')
                if occluded:
                    d.rounded_rectangle((48,148,720,194),radius=6,fill='#9c3b35')
                    d.text((65,159),'MARKER HIDDEN: laptop sends zero commands',font=font(20,True),fill='white')
                emit(im, preview=step % 16 == 0)
                if step == 120:
                    emit(card('How the moving system works', [
                        ('01 / Observe', 'The camera supplies marker position and heading to the laptop.'),
                        ('02 / Control', 'Adaptive motion bursts guide the robot directly toward the selected goal.'),
                        ('03 / Offload', 'The Pi worker compares control profiles and returns advice independently.'),
                    ]), 4*FPS)
                if result and world.t>8 and not screenshot_saved:
                    im.save(a.out/'simulation-preview.png'); screenshot_saved=True
            if state.status == 'arrived': reached = True; emit(im,2*FPS); break
        if not reached: raise RuntimeError('demo did not arrive within simulated time budget')
        if not saw_lost or not zero_lost: raise RuntimeError('lost-marker scene did not emit zero commands')
        if result is None: raise RuntimeError('background HTTP worker did not complete')
    finally:
        client.close(); server.shutdown(); server.server_close(); thread.join(timeout=1)
    error = dist((p.x,p.y),goal)
    emit(card('Navigation and background computation', [
        ('Navigation', f'Goal reached in {world.t:.1f} simulated seconds; true final error {error*1000:.0f} mm.'),
        ('Lost-marker response', 'When the generated marker is hidden, the laptop emits zero wheel commands.'),
        ('Offload boundary', 'An authenticated HTTP job finishes independently; advice never drives the motors.'),
    ]), 4*FPS)
    emit(card('Evaluation highlights', [
        ('Delayed camera / adaptive burst control', '10/10 tests passed with a 0.3 s camera delay.'),
        ('Communication uncertainty', '10/10 tests passed with 15% simulated packet loss.'),
        ('Online adaptation', '10/10 waypoint courses completed from poor initial motion estimates.'),
    ]), 4*FPS)
    encoder.close()
    if not previews: raise RuntimeError('no preview frames')
    # The GIF is a short, downsampled excerpt, not a second independent experiment.
    previews[0].save(a.out/'simulation-preview.gif',save_all=True,append_images=previews[1:],
        duration=240,loop=0,optimize=True)
    with (a.out/'demo-trajectory.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(trace[0]));writer.writeheader();writer.writerows(trace)
    result['host'] = 'local simulation worker'
    (a.out/'demo-worker-result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    summary={'mode':'software simulation; no hardware used','seed':7,'firmware_bot_id':2,
        'printed_marker_id':1,'goal_m':list(goal),'simulated_duration_s':world.t,
        'final_true_pose_m_rad':[p.x,p.y,p.theta],'final_error_m':error,'reached':reached,
        'lost_marker_zero_commands':zero_lost,'http_worker_completed':True,
        'worker_execution_host':'laptop, not physical Raspberry Pi',
        'video_fps':FPS,'video_frames':frames_written,'video_duration_s':frames_written/FPS}
    (a.out/'demo-summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__ == '__main__': main()
