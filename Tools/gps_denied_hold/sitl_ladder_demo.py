#!/usr/bin/env python3
"""
SITL demonstration of the GPS-denied hold mode's health ladder.

Runs PX4 SITL with the built-in SIH quadrotor (no external simulator), flies the
custom mode (nav state MY_NEW_MODE) with MAVLink sticks, and injects GPS faults:

  takeoff -> hover -> constant-velocity cruise -> hover
  -> GPS "wrong" (position jump, a crude spoof) -> GPS ok -> hover
  -> GPS off (dead reckoning) -> GPS ok -> hover -> land

Outputs in --out: px4_console.log, phases.json (phase start times in PX4 boot
time) and the flight ULog, which plot_ladder_log.py turns into a figure and a
transition table.

    make px4_sitl_default
    python3 Tools/gps_denied_hold/sitl_ladder_demo.py --out /tmp/gdh_run
    python3 Tools/gps_denied_hold/plot_ladder_log.py /tmp/gdh_run
"""

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

PX4_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


class Link(threading.Thread):
    """One I/O thread: sends MANUAL_CONTROL at 20 Hz and drains incoming messages."""

    def __init__(self, url):
        super().__init__(daemon=True)
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.sticks = [0, 0, 500, 0]  # x pitch, y roll, z throttle (0..1000, 500 = centre), r yaw
        self.boot_ms = 0
        self.armed = False
        self.custom_mode = 0
        self.statustext = []
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        next_send = 0.0
        while self.running:
            now = time.time()
            if now >= next_send and self.m.target_system:
                x, y, z, r = self.sticks
                self.m.mav.manual_control_send(self.m.target_system, x, y, z, r, 0)
                next_send = now + 0.05
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            t = msg.get_type()
            if hasattr(msg, "time_boot_ms"):
                self.boot_ms = max(self.boot_ms, msg.time_boot_ms)
            if t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.m.target_system = msg.get_srcSystem()
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.custom_mode = msg.custom_mode
            elif t == "STATUSTEXT":
                with self.lock:
                    self.statustext.append((self.boot_ms, msg.text))


def px4_cmd(build, *args, timeout=15):
    exe = os.path.join(build, "bin", f"px4-{args[0]}")
    r = subprocess.run([exe, *args[1:]], cwd=os.path.join(build, "rootfs"),
                       capture_output=True, text=True, timeout=timeout)
    return (r.stdout + r.stderr).strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", default=os.path.join(PX4_ROOT, "build", "px4_sitl_default"))
    ap.add_argument("--out", default="gdh_run")
    ap.add_argument("--speed", type=float, default=1.0, help="time scale for phase durations (debug)")
    a = ap.parse_args()

    build = os.path.abspath(a.build)
    rootfs = os.path.join(build, "rootfs")
    out = os.path.abspath(a.out)
    os.makedirs(out, exist_ok=True)
    os.makedirs(rootfs, exist_ok=True)
    logs_before = set(glob.glob(os.path.join(rootfs, "log", "*", "*.ulg")))

    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIMULATOR="sihsim")
    console = open(os.path.join(out, "px4_console.log"), "w")
    px4 = subprocess.Popen([os.path.join(build, "bin", "px4"), "-d"], cwd=rootfs, env=env,
                           stdout=console, stderr=subprocess.STDOUT)
    phases = []
    link = None

    def phase(name, seconds=0.0):
        phases.append({"name": name, "t_s": link.boot_ms / 1000.0})
        print(f"[{link.boot_ms / 1000.0:7.1f} s] {name}", flush=True)
        if seconds:
            time.sleep(seconds * a.speed)

    try:
        link = Link("udpin:0.0.0.0:14550")
        link.start()
        t0 = time.time()
        while not link.m.target_system:
            if time.time() - t0 > 60 or px4.poll() is not None:
                sys.exit("no heartbeat from PX4 SITL")
            time.sleep(0.2)

        print(px4_cmd(build, "param", "set", "COM_RC_IN_MODE", "1"))
        print(px4_cmd(build, "param", "set", "SYS_FAILURE_EN", "1"))
        print(px4_cmd(build, "param", "set", "NAV_DLL_ACT", "0"))
        time.sleep(2)

        # wait for EKF/GPS, switch to the custom mode, then arm
        t0 = time.time()
        while not link.armed:
            if time.time() - t0 > 120:
                sys.exit("could not arm")
            px4_cmd(build, "commander", "mode", "my_new_mode")
            time.sleep(1)
            px4_cmd(build, "commander", "arm")
            time.sleep(2)

        phase("armed in GPS-denied hold")
        link.sticks = [0, 0, 850, 0]
        phase("takeoff (throttle stick up)", 5)
        link.sticks = [0, 0, 500, 0]
        phase("hover, GPS healthy", 15)
        link.sticks = [500, 0, 500, 0]
        phase("cruise: pitch stick forward", 12)
        link.sticks = [0, 0, 500, 0]
        phase("hover", 12)
        print(px4_cmd(build, "failure", "gps", "wrong"))
        phase("GPS spoof: position jump (failure gps wrong)", 15)
        print(px4_cmd(build, "failure", "gps", "ok"))
        phase("GPS restored", 20)
        print(px4_cmd(build, "failure", "gps", "off"))
        phase("GPS off: dead reckoning", 12)
        print(px4_cmd(build, "failure", "gps", "ok"))
        phase("GPS restored", 25)
        px4_cmd(build, "commander", "land")
        phase("land")
        t0 = time.time()
        while link.armed and time.time() - t0 < 60:
            time.sleep(1)
        phase("disarmed" if not link.armed else "land timeout")
        time.sleep(3)

    finally:
        if link is not None:
            with open(os.path.join(out, "phases.json"), "w") as f:
                json.dump({"phases": phases, "statustext": link.statustext}, f, indent=1)
            link.running = False
        try:
            px4_cmd(build, "shutdown", timeout=10)
        except Exception:
            pass
        try:
            px4.wait(timeout=15)
        except subprocess.TimeoutExpired:
            px4.kill()
        console.close()

    new_logs = sorted(set(glob.glob(os.path.join(rootfs, "log", "*", "*.ulg"))) - logs_before, key=os.path.getmtime)
    if not new_logs:
        sys.exit("no ULog written")
    shutil.copy(new_logs[-1], os.path.join(out, "flight.ulg"))
    print(f"wrote {out}/flight.ulg, phases.json, px4_console.log")


if __name__ == "__main__":
    main()
