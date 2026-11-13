#!/usr/bin/env python3
"""
Plot and tabulate a GPS-denied hold SITL run (output directory of sitl_ladder_demo.py).

    python3 Tools/gps_denied_hold/plot_ladder_log.py RUN_DIR [--png out.png]

Prints the ladder transitions (time, from -> to, reason bits) and the hover
detector statistics per phase, and writes RUN_DIR/ladder.png (or --png).
"""

import argparse
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from pyulog import ULog  # noqa: E402

LEVELS = ["HEALTHY", "DEGRADED", "FAILING", "LOST"]
REASONS = {1: "NO_POS_AIDING", 2: "NO_VEL_AIDING", 4: "DEAD_RECKONING", 8: "TEST_RATIO_HIGH",
           16: "REJECTING", 32: "EPH_HIGH", 64: "XY_INVALID", 128: "POS_RESET", 256: "DR_TIMEOUT",
           512: "RANGE_RANK1", 1024: "ESTIMATOR_STALE"}
NAV_NAMES = {2: "POSCTL", 1: "ALTCTL", 16: "MY_NEW_MODE", 18: "AUTO_LAND", 12: "DESCEND", 4: "AUTO_LOITER"}


def topic(ulog, name):
    try:
        d = ulog.get_dataset(name).data
    except (KeyError, IndexError, ValueError):
        return None
    d = dict(d)
    d["t"] = d["timestamp"] / 1e6
    return d


def reasons_text(bits):
    return "|".join(n for b, n in REASONS.items() if bits & b) or "-"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    ulog = ULog(os.path.join(a.run, "flight.ulg"))
    with open(os.path.join(a.run, "phases.json")) as f:
        phases = json.load(f)["phases"]

    st = topic(ulog, "gps_denied_hold_status")
    vs = topic(ulog, "vehicle_status")
    lp = topic(ulog, "vehicle_local_position")
    fl = topic(ulog, "estimator_status_flags")
    if st is None:
        raise SystemExit("no gps_denied_hold_status in the log")

    # transitions
    print("Ladder transitions (PX4 boot time)")
    print(f"{'t [s]':>8}  {'from':9s} -> {'to':9s}  reasons")
    h = st["health"]
    for i in np.nonzero(np.diff(h.astype(int)))[0] + 1:
        print(f"{st['t'][i]:8.1f}  {LEVELS[h[i - 1]]:9s} -> {LEVELS[h[i]]:9s}  {reasons_text(int(st['reasons'][i]))}")

    # mode changes
    if vs is not None:
        print("\nNavigation state changes")
        ns = vs["nav_state"]
        for i in [0] + list(np.nonzero(np.diff(ns.astype(int)))[0] + 1):
            print(f"{vs['t'][i]:8.1f}  nav_state {int(ns[i])} {NAV_NAMES.get(int(ns[i]), '')}")

    # hover detector per phase (item 1 evidence)
    print("\nHover detector by phase: fraction of samples flagged hover, mean |a_xy| filtered, mean speed")
    spd_t = lp["t"] if lp is not None else None
    spd = np.hypot(lp["vx"], lp["vy"]) if lp is not None else None
    bounds = [p["t_s"] for p in phases] + [st["t"][-1]]
    for p, t1 in zip(phases, bounds[1:]):
        m = (st["t"] >= p["t_s"]) & (st["t"] < t1)
        if not m.any():
            continue
        v = np.nan
        if spd is not None:
            mv = (spd_t >= p["t_s"]) & (spd_t < t1)
            v = spd[mv].mean() if mv.any() else np.nan
        print(f"  {p['name'][:44]:44s} hover {st['hover'][m].mean():4.2f}  |a| {st['accel_xy_filt'][m].mean():5.2f} m/s2"
              f"  speed {v:5.2f} m/s")

    # accelerometer-only criterion (no stick gating): what an accelerometer alone would call "hover"
    print("\nAccelerometer-only criterion |a_xy| < 0.3 m/s^2 (no stick gating), fraction of samples per phase")
    for p, t1 in zip(phases, bounds[1:]):
        m = (st["t"] >= p["t_s"] + 3.0) & (st["t"] < t1)  # skip the first 3 s (transient)
        if m.any():
            print(f"  {p['name'][:44]:44s} {np.mean(st['accel_xy_filt'][m] < 0.3):4.2f}")

    fig, ax = plt.subplots(6, 1, figsize=(7.2, 9.5), sharex=True, constrained_layout=True,
                           gridspec_kw={"height_ratios": [1.3, 1, 1, 1, 1, 0.8]})
    ax[0].step(st["t"], st["health"], where="post", color="#e34948", lw=2, label="ladder level")
    ax[0].step(st["t"], st["candidate"], where="post", color="#555555", lw=0.9, ls="--", label="candidate (before hysteresis)")
    ax[0].set_yticks(range(4), LEVELS)
    ax[0].set_ylim(-0.4, 3.4)
    ax[0].invert_yaxis()
    ax[0].legend(loc="lower left", fontsize=7, framealpha=0.9)
    ax[0].set_title("GPS-denied hold: navigation-health ladder (PX4 SITL, SIH quadrotor)", fontsize=10)
    if vs is not None:
        in_mode = vs["nav_state"] == 16
        t_first = vs["t"][in_mode][0] if in_mode.any() else vs["t"][0]
        fs = (~in_mode) & (vs["t"] > t_first)
        ax[0].fill_between(vs["t"], -0.4, 3.4, where=fs, step="post", color="#999999", alpha=0.2, lw=0)
        ax[0].text(vs["t"][fs][0] if fs.any() else 0, -0.3, "  commander\n  failsafe", fontsize=7, color="#555555", va="top")

    ax[1].semilogy(st["t"], np.maximum(st["pos_test_ratio"], 1e-2), color="#2a78d6")
    ax[1].axhline(1.0, color="#222222", lw=0.8, ls=":")
    ax[1].set_ylim(5e-3, 1e9)
    ax[1].text(st["t"][0], 1.5, " EKF gate (ratio 1)", fontsize=7, color="#555555")
    ax[1].set_ylabel("GNSS pos.\ntest ratio")

    ax[2].plot(st["t"], st["eph"], color="#1baf7a")
    ax[2].set_ylabel("eph [m]")

    ax[3].plot(st["t"], st["accel_xy_filt"], color="#6250d6")
    ax[3].axhline(0.3, color="#222222", lw=0.8, ls=":")
    ax[3].fill_between(st["t"], 0, st["hover"] * 1.0, step="post", color="#1baf7a", alpha=0.25, lw=0, label="hover flag")
    ax[3].set_ylim(0, 1.0)
    ax[3].set_ylabel("|a_xy| LPF\n[m/s²]")
    ax[3].legend(loc="upper right", fontsize=7)

    if lp is not None:
        ax[4].plot(lp["t"], spd, color="#eb6834")
    ax[4].set_ylabel("speed [m/s]")

    if fl is not None:
        ax[5].step(fl["t"], fl["cs_gnss_pos"] * 0.8 + 2, where="post", color="#2a78d6")
        ax[5].step(fl["t"], fl["cs_inertial_dead_reckoning"] * 0.8 + 1, where="post", color="#eb6834")
    if lp is not None:
        ax[5].step(lp["t"], lp["xy_valid"] * 0.8, where="post", color="#1baf7a")
    ax[5].set_yticks([0.4, 1.4, 2.4], ["xy valid", "dead reck.", "GNSS pos"], fontsize=7)
    ax[5].set_xlabel("PX4 time since boot [s]")

    for p in phases:
        for x in ax:
            x.axvline(p["t_s"], color="#cccccc", lw=0.8)
    print("\nPhase markers in the figure")
    for i, p in enumerate(phases, 1):
        ax[0].text(p["t_s"], -0.38, str(i), fontsize=7, ha="left", va="top", color="#222222")
        print(f"  phase {i}: {p['t_s']:6.1f} s  {p['name']}")

    png = a.png or os.path.join(a.run, "ladder.png")
    fig.savefig(png, dpi=150)
    print(f"\nwrote {png}")


if __name__ == "__main__":
    main()
