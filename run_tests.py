#!/usr/bin/env python3
"""Open-loop manoeuvre tests for BlueROV models: Gazebo (Jetty or Harmonic) + ArduSub SITL.

Each trial:
  1. starts a headless Gazebo server with the layout's world and ArduSub SITL (JSON backend, lock-step),
  2. arms and plays the mission (fixed MANUAL_CONTROL stick inputs), timing every phase in
     simulation time (SITL boot time)
  3. after shutdown, reads the SITL dataflash log (.BIN) and computes per-phase metrics from the
     simulator's ground truth (SIM/SIM2 messages)

Usage:  python3 run_tests.py [--layout bluerov1|bluerov2|bluerov2-heavy] [--trials N] [--config cfg.json]
"""

import argparse
import csv
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent
AP = ROOT / "ardupilot"

CONFIG = {
    "layout": "bluerov1",  # bluerov1 | bluerov2 | bluerov2-heavy
    "trials": 1,  # physics is deterministic; >1 only measures timing jitter
    "mode": "STABILIZE",  # MANUAL | STABILIZE | ALT_HOLD
    "rate_hz": 20,  # MANUAL_CONTROL send rate (wall clock)
    "preflight_s": 10,  # sim seconds to let SITL/EKF initialise before arming
    # Physical overrides applied to a per-run copy of the model (None = keep model values).
    "mass_kg": None,  # inertia is scaled with the mass; buoyancy is unchanged
    "com_offset_m": None,  # [x, y, z] added to the CoM, Gazebo body frame (x fwd, y left, z up)
    # Mission steps: [label, x, y, z, r, sim_seconds]
    # x/y/r: -1000..1000, z: 0..1000 with 500 = neutral (250 = dive, 750 = rise)
    "mission": [
        ["settle", 0, 0, 500, 0, 5],
        ["surge", 500, 0, 500, 0, 4],
        ["coast_surge", 0, 0, 500, 0, 6],
        ["sway", 0, 500, 500, 0, 4],
        ["coast_sway", 0, 0, 500, 0, 6],
        ["dive", 0, 0, 250, 0, 4],
        ["coast_dive", 0, 0, 500, 0, 6],
        ["rise", 0, 0, 750, 0, 4],
        ["coast_rise", 0, 0, 500, 0, 6],
        ["yaw", 0, 0, 500, 500, 3],
        ["coast_yaw", 0, 0, 500, 0, 6],
    ],
    "plugin_path": str(ROOT / "ardupilot_gazebo" / "build"), # folder containing libArduPilotPlugin.so
    "results_dir": str(ROOT / "results"),
}

# layout -> Gazebo model, ArduPilot defaults file, FRAME_CONFIG, motor count
LAYOUTS = {
    "bluerov1": {"model": "bluerov1", "parm": "sub.parm", "frame": 0, "motors": 6},
    "bluerov2": {"model": "bluerov2", "parm": "sub.parm", "frame": 1, "motors": 6},
    "bluerov2-heavy": {
        "model": "bluerov2_heavy",
        "parm": "sub-6dof.parm",
        "frame": 2,
        "motors": 8,
    },
}

# Extra SITL parameters (loaded after the layout's defaults file)
PARAMS = """FRAME_CONFIG {frame}
LOG_BITMASK 180223
LOG_FILE_DSRMROT 1
"""  # 180223 = sub.parm default + ATTITUDE_FAST (25 Hz SIM attitude); close the log on disarm

HOME = "-35.1192416,150.7296753,0.0,0"  # lat,lon,alt,heading
SAT_PWM = 350  # |pwm - 1500| >= this counts as near saturation (1100..1900)


# ---------- setup ----------


def check_setup(cfg):
    errors = []
    if not shutil.which("gz"):
        errors.append(
            "`gz` not found: install Gazebo Jetty (sudo apt install gz-jetty)."
        )
    if not (AP / "build/sitl/bin/ardusub").exists():
        errors.append(
            f"ArduSub SITL not built: cd {AP} && ./waf configure --board sitl && ./waf sub"
        )
    if not any(
        (Path(p) / "libArduPilotPlugin.so").exists()
        for p in cfg["plugin_path"].split(os.pathsep)
    ):
        errors.append(
            f"libArduPilotPlugin.so not found in plugin_path={cfg['plugin_path']} "
            "(build github.com/ArduPilot/ardupilot_gazebo with GZ_VERSION=jetty)."
        )
    if errors:
        sys.exit("Setup problems:\n  " + "\n  ".join(errors))


def prepare_models(cfg, run_dir):
    """Return a models dir to put first on GZ_SIM_RESOURCE_PATH (overridden copy), or None."""
    if cfg["mass_kg"] is None and cfg["com_offset_m"] is None:
        return None
    name = LAYOUTS[cfg["layout"]]["model"]
    src, dst = ROOT / "models" / name, run_dir / "models" / name
    shutil.copytree(src, dst, ignore=shutil.ignore_patterns("meshes"))
    (dst / "meshes").symlink_to(src / "meshes")
    tree = ET.parse(dst / "model.sdf")
    inertial = tree.find("model/link[@name='base_link']/inertial")
    if cfg["mass_kg"] is not None:
        mass = inertial.find("mass")
        k = cfg["mass_kg"] / float(mass.text)
        mass.text = str(cfg["mass_kg"])
        for tag in ("ixx", "iyy", "izz"):
            el = inertial.find(f"inertia/{tag}")
            el.text = f"{float(el.text) * k:.6g}"
    if cfg["com_offset_m"] is not None:
        pose = inertial.find("pose")
        if pose is None:
            pose = ET.SubElement(inertial, "pose")
            pose.text = "0 0 0 0 0 0"
        v = [float(x) for x in pose.text.split()]
        v[:3] = [a + b for a, b in zip(v[:3], cfg["com_offset_m"])]
        pose.text = " ".join(f"{x:.6g}" for x in v)
    tree.write(dst / "model.sdf", xml_declaration=True, encoding="UTF-8")
    print(f"Model override written to {dst / 'model.sdf'}")
    return dst.parent


# ---------- processes ----------


def spawn(name, cmd, log_path, **kw):
    log = open(log_path, "w")
    p = subprocess.Popen(
        cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, **kw
    )
    p.name, p.log = name, log
    return p


def stop(p):
    if p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
    p.log.close()


def start_gazebo(cfg, tdir, override_dir, k):
    paths = lambda *ps: os.pathsep.join(str(p) for p in ps if p)
    env = dict(
        os.environ,
        GZ_IP="127.0.0.1",
        GZ_PARTITION=f"rov_sim_{os.getpid()}_{k}",
        GZ_SIM_SYSTEM_PLUGIN_PATH=paths(
            cfg["plugin_path"], os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH")
        ),
        GZ_SIM_RESOURCE_PATH=paths(
            override_dir,
            ROOT / "models",
            ROOT / "worlds",
            os.environ.get("GZ_SIM_RESOURCE_PATH"),
        ),
    )
    world = ROOT / "worlds" / f"{LAYOUTS[cfg['layout']]['model']}_underwater.world"
    print(f"  watch with: GZ_IP=127.0.0.1 GZ_PARTITION={env['GZ_PARTITION']} gz sim -g")
    return spawn(
        "gazebo",
        ["gz", "sim", "-s", "-r", "-v", "3", str(world)],
        tdir / "gazebo.log",
        env=env,
    )


def start_sitl(cfg, tdir):
    lay = LAYOUTS[cfg["layout"]]
    (tdir / "overrides.parm").write_text(PARAMS.format(frame=lay["frame"]))
    defaults = f"{AP / 'Tools/autotest/default_params' / lay['parm']},{tdir / 'overrides.parm'}"
    cmd = [
        str(AP / "build/sitl/bin/ardusub"),
        "--model",
        "JSON",
        "-I0",  # MAVLink tcp:5760, JSON udp:9002
        "--defaults",
        defaults,
        "--home",
        HOME,
    ]
    return spawn(
        "ardusub", cmd, tdir / "sitl.log", cwd=tdir
    )  # .BIN logs land in tdir/logs


# ---------- flying the mission ----------


class Vehicle:
    def __init__(self, procs, rate_hz):
        self.procs, self.period = procs, 1.0 / rate_hz
        self.mav = mavutil.mavlink_connection(
            "tcp:127.0.0.1:5760", source_system=255, retries=30
        )
        hb = self.mav.wait_heartbeat(timeout=30)
        if hb is None:
            raise TimeoutError("No heartbeat from ArduSub on tcp:127.0.0.1:5760")
        self.mav.target_component = hb.get_srcComponent()
        self.boot_ms, self.armed, self.mode, self.last_hb = 0, False, None, 0.0
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
            1e6 / rate_hz,
            0,
            0,
            0,
            0,
            0,
        )

    def tick(self, x=0, y=0, z=500, r=0):
        """Send one stick command (+ 1 Hz GCS heartbeat), process incoming messages, sleep."""
        start = time.monotonic()
        for p in self.procs:
            if p.poll() is not None:
                raise RuntimeError(
                    f"{p.name} exited (code {p.returncode}); see {p.log.name}"
                )
        m = self.mav.mav
        m.manual_control_send(self.mav.target_system, x, y, z, r, 0)
        if start - self.last_hb >= 1.0:
            m.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            self.last_hb = start
        while (msg := self.mav.recv_match(blocking=False)) is not None:
            t = msg.get_type()
            if hasattr(msg, "time_boot_ms"):
                self.boot_ms = max(self.boot_ms, msg.time_boot_ms)
            if (
                t == "HEARTBEAT"
                and msg.get_srcSystem() == self.mav.target_system
                and msg.get_srcComponent() == self.mav.target_component
            ):
                self.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.mode = msg.custom_mode
            elif t == "STATUSTEXT":
                print("  [ardusub]", msg.text)
        time.sleep(max(0.0, self.period - (time.monotonic() - start)))

    def wait_for(self, cond, timeout, what, **sticks):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                raise TimeoutError(f"Timed out waiting for {what}")
            self.tick(**sticks)

    def hold(self, sim_s, x=0, y=0, z=500, r=0):
        """Hold sticks for sim_s seconds of simulation time; return (t0_ms, t1_ms)."""
        t0 = self.boot_ms
        last, last_change = t0, time.monotonic()
        while self.boot_ms < t0 + sim_s * 1000:
            self.tick(x, y, z, r)
            if self.boot_ms != last:
                last, last_change = self.boot_ms, time.monotonic()
            elif time.monotonic() - last_change > 10:
                raise TimeoutError(
                    "Simulation time stopped advancing (Gazebo/SITL link lost?)"
                )
        return t0, self.boot_ms

    def arm(self, on):
        # 21196 = force: skips pre-arm checks (simulation only)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if on else 0,
            21196,
            0,
            0,
            0,
            0,
            0,
        )
        self.wait_for(lambda: self.armed == on, 15, "arming" if on else "disarming")


def fly(veh, cfg):
    veh.wait_for(
        lambda: veh.boot_ms > 0,
        60,
        "simulation time (is Gazebo running and the ArduPilot plugin connected? see gazebo.log)",
    )
    veh.wait_for(lambda: veh.boot_ms >= cfg["preflight_s"] * 1000, 120, "preflight")
    mode_id = veh.mav.mode_mapping()[cfg["mode"]]
    veh.mav.set_mode(mode_id)
    veh.wait_for(lambda: veh.mode == mode_id, 10, f"mode {cfg['mode']}")
    veh.arm(True)
    phases = []
    for label, x, y, z, r, secs in cfg["mission"]:
        print(f"  {label:<12} x={x} y={y} z={z} r={r} for {secs}s")
        phases.append((label, *veh.hold(secs, x, y, z, r)))
    veh.arm(False)
    veh.hold(2)  # let the logger close the file
    return phases


# ---------- log analysis ----------


def read_log(tdir):
    bins = sorted((tdir / "logs").glob("*.BIN"))
    if not bins:
        raise FileNotFoundError(f"No .BIN log in {tdir / 'logs'}")
    log = mavutil.mavlink_connection(str(bins[-1]))
    data = {"SIM": [], "SIM2": [], "RCOU": []}
    while (m := log.recv_match(type=list(data))) is not None:
        data[m.get_type()].append(m.to_dict())
    for rows in data.values():
        for r in rows:
            r["t"] = r["TimeUS"] / 1e6
    return data


def window(rows, t0, t1):
    return [r for r in rows if t0 <= r["t"] < t1]


def wrap180(a):
    return (a + 180) % 360 - 180


def phase_metrics(data, phases, n_motors):
    out = []
    for label, t0_ms, t1_ms in phases:
        t0, t1 = t0_ms / 1000, t1_ms / 1000
        pos, att, rc = (
            window(data["SIM2"], t0, t1),
            window(data["SIM"], t0, t1),
            window(data["RCOU"], t0, t1),
        )
        if not pos or not att:
            continue
        a, b = pos[0], pos[-1]
        yaw0 = math.radians(att[0]["Yaw"])
        dn, de = b["PN"] - a["PN"], b["PE"] - a["PE"]
        speed = lambda r: math.sqrt(r["VN"] ** 2 + r["VE"] ** 2 + r["VD"] ** 2)
        sat = sum(
            any(abs(r[f"C{i}"] - 1500) >= SAT_PWM for i in range(1, n_motors + 1))
            for r in rc
        )
        out.append(
            dict(
                phase=label,
                duration_s=round(t1 - t0, 2),
                d_forward_m=round(dn * math.cos(yaw0) + de * math.sin(yaw0), 3),
                d_right_m=round(-dn * math.sin(yaw0) + de * math.cos(yaw0), 3),
                d_depth_m=round(b["PD"] - a["PD"], 3),  # positive = deeper
                d_yaw_deg=round(
                    sum(wrap180(q["Yaw"] - p["Yaw"]) for p, q in zip(att, att[1:])), 1
                ),
                max_abs_roll_deg=round(max(abs(r["Roll"]) for r in att), 2),
                max_abs_pitch_deg=round(max(abs(r["Pitch"]) for r in att), 2),
                peak_speed_mps=round(max(map(speed, pos)), 3),
                end_speed_mps=round(speed(b), 3),  # ~0 => coast long enough
                near_sat_pct=round(100 * sat / len(rc), 1) if rc else None,
            )
        )
    return out


def write_truth_csv(data, phases, path):
    """Ground truth at SIM2 rate with the latest SIM attitude and the phase label."""
    att, j = data["SIM"], 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_s",
                "phase",
                "north_m",
                "east_m",
                "down_m",
                "vn",
                "ve",
                "vd",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
            ]
        )
        for r in data["SIM2"]:
            while j + 1 < len(att) and att[j + 1]["t"] <= r["t"]:
                j += 1
            phase = next(
                (p for p, t0, t1 in phases if t0 / 1000 <= r["t"] < t1 / 1000), ""
            )
            a = att[j] if att else {"Roll": None, "Pitch": None, "Yaw": None}
            w.writerow(
                [
                    round(r["t"], 4),
                    phase,
                    round(r["PN"], 4),
                    round(r["PE"], 4),
                    round(r["PD"], 4),
                    round(r["VN"], 4),
                    round(r["VE"], 4),
                    round(r["VD"], 4),
                    a["Roll"],
                    a["Pitch"],
                    a["Yaw"],
                ]
            )


def plot(trials, run_dir):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot")
        return
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    for k, (data, phases) in trials.items():
        t_start = phases[0][1] / 1000
        pos = [r for r in data["SIM2"] if r["t"] >= t_start]
        att = [r for r in data["SIM"] if r["t"] >= t_start]
        ax[0].plot(
            [r["t"] - t_start for r in pos], [r["PD"] for r in pos], label=f"trial {k}"
        )
        ax[1].plot(
            [r["t"] - t_start for r in pos], [math.hypot(r["VN"], r["VE"]) for r in pos]
        )
        ax[2].plot([r["t"] - t_start for r in att], [r["Roll"] for r in att])
        ax[3].plot([r["t"] - t_start for r in att], [r["Pitch"] for r in att])
    data, phases = next(iter(trials.values()))
    for a in ax:
        for label, t0, _ in phases:
            a.axvline(t0 / 1000 - phases[0][1] / 1000, color="grey", lw=0.5)
    for label, t0, _ in phases:
        ax[0].text(
            t0 / 1000 - phases[0][1] / 1000,
            0,
            label,
            rotation=90,
            fontsize=7,
            va="bottom",
            transform=ax[0].get_xaxis_transform(),
        )
    ax[0].set_ylabel("depth (m)")
    ax[0].invert_yaxis()
    ax[0].legend()
    ax[1].set_ylabel("horiz. speed (m/s)")
    ax[2].set_ylabel("roll (deg)")
    ax[3].set_ylabel("pitch (deg)")
    ax[3].set_xlabel("simulation time since mission start (s)")
    fig.suptitle(f"{run_dir.name}: simulator ground truth")
    fig.tight_layout()
    fig.savefig(run_dir / "truth.png", dpi=150)


# ---------- main ----------


def run_trial(cfg, run_dir, override_dir, k):
    tdir = run_dir / f"trial_{k:02d}"
    tdir.mkdir()
    procs = []
    try:
        procs.append(start_gazebo(cfg, tdir, override_dir, k))
        procs.append(start_sitl(cfg, tdir))
        phases = fly(Vehicle(procs, cfg["rate_hz"]), cfg)
    finally:
        for p in reversed(procs):
            stop(p)
    data = read_log(tdir)
    write_truth_csv(data, phases, tdir / "truth.csv")
    return data, phases


def print_summary(summary, keys):
    print(f"\n{'phase':<13}" + "".join(f"{k:>18}" for k in keys))
    for phase in dict.fromkeys(m["phase"] for m in summary):
        ms = [m for m in summary if m["phase"] == phase]
        cells = []
        for k in keys:
            vals = [m[k] for m in ms if m[k] is not None]
            if not vals:
                cells.append(f"{'-':>18}")
            elif len(vals) == 1:
                cells.append(f"{vals[0]:>18.3f}")
            else:
                cells.append(
                    f"{statistics.mean(vals):>10.3f} ±{statistics.stdev(vals):<6.3f}"
                )
        print(f"{phase:<13}" + "".join(cells))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config", help="JSON file with CONFIG overrides")
    ap.add_argument("--layout", choices=LAYOUTS)
    ap.add_argument("--trials", type=int)
    args = ap.parse_args()
    cfg = dict(CONFIG)
    if args.config:
        cfg.update(json.loads(Path(args.config).read_text()))
    if args.layout:
        cfg["layout"] = args.layout
    if args.trials:
        cfg["trials"] = args.trials
    check_setup(cfg)

    run_dir = (
        Path(cfg["results_dir"]) / f"{cfg['layout']}_{datetime.now():%Y%m%d_%H%M%S}"
    )
    run_dir.mkdir(parents=True)
    (run_dir / "config.json").write_text(json.dumps(cfg, indent=2))
    override_dir = prepare_models(cfg, run_dir)
    n_motors = LAYOUTS[cfg["layout"]]["motors"]

    trials, summary = {}, []
    for k in range(1, cfg["trials"] + 1):
        print(f"== {cfg['layout']} trial {k}/{cfg['trials']} ==")
        try:
            trials[k] = run_trial(cfg, run_dir, override_dir, k)
        except Exception as e:
            print(f"  trial {k} failed: {e!r}")
            continue
        summary += [dict(trial=k, **m) for m in phase_metrics(*trials[k], n_motors)]
        with open(
            run_dir / "summary.csv", "w", newline=""
        ) as f:  # rewritten after every trial
            w = csv.DictWriter(f, summary[0].keys())
            w.writeheader()
            w.writerows(summary)

    if not summary:
        sys.exit(f"No successful trials; see logs in {run_dir}")
    print(f"\nResults: {run_dir}")
    print_summary(
        summary,
        [
            "d_forward_m",
            "d_right_m",
            "d_depth_m",
            "d_yaw_deg",
            "max_abs_roll_deg",
            "max_abs_pitch_deg",
            "end_speed_mps",
            "near_sat_pct",
        ],
    )
    plot(trials, run_dir)


if __name__ == "__main__":
    main()
