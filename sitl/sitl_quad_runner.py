"""Run a guidance law on an ArduCopter SITL quadcopter.

Frames: this repo uses x = east, y = north. MAVLink LOCAL_POSITION_NED uses
x = north, y = east, z = down. The conversion is marked (NED) below.

SAFETY: simulation only. Do NOT point this at a real aircraft without a safety pilot,
a geofence, a tested failsafe, and a review of every line.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ctrack.guidance import LAW_NAMES, make_law  # noqa: E402
from ctrack.route import build_route, standard_route  # noqa: E402
from ctrack.sitl_control import finished, save_run, velocity_command  # noqa: E402
from ctrack.vehicles import QUADCOPTER  # noqa: E402

LOCAL_POSITION_NED_ID = 32
COPTER_GUIDED, COPTER_LAND = 4, 9        # ArduCopter flight-mode numbers


class Link:
    """Reads EVERY message (nothing is thrown away) and keeps the latest vehicle state.

    It also works out which system is the vehicle from its heartbeat and points all
    commands at it. Ground-station heartbeats (autopilot = INVALID) are ignored.
    """

    def __init__(self, m, mavutil):
        self.m, self.mv = m, mavutil
        self.veh_sys = self.veh_comp = None
        self.custom_mode = None
        self.armed = False
        self.pos = None
        self.n_pos = 0
        self.acks = {}
        self.params = {}

    def poll(self, timeout=0.2):
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        t = msg.get_type()
        mav = self.mv.mavlink
        if t == "HEARTBEAT":
            if msg.autopilot != mav.MAV_AUTOPILOT_INVALID:
                self.veh_sys, self.veh_comp = msg.get_srcSystem(), msg.get_srcComponent()
                self.custom_mode = msg.custom_mode
                self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                self.m.target_system, self.m.target_component = self.veh_sys, self.veh_comp
        elif t == "STATUSTEXT":
            text = msg.text.decode(errors="ignore") if isinstance(msg.text, bytes) else msg.text
            print("  [ArduPilot]", text.strip("\x00"))
        elif t == "COMMAND_ACK":
            self.acks[msg.command] = msg.result
        elif t == "LOCAL_POSITION_NED":
            self.pos = msg
            self.n_pos += 1
        elif t == "PARAM_VALUE":
            pid = msg.param_id.decode(errors="ignore") if isinstance(msg.param_id, bytes) else str(msg.param_id)
            self.params[pid.strip("\x00")] = msg.param_value
        return msg

    def pump(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.poll(min(0.2, max(0.0, t_end - time.time())))

    def wait_until(self, cond, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.poll(0.2)
            if cond():
                return True
        return cond()

    def wait_vehicle(self, seconds=30):
        if not self.wait_until(lambda: self.veh_sys is not None, seconds):
            raise RuntimeError("no heartbeat from a vehicle within %d s. Is the simulator running and "
                               "forwarding to this address (--connect)?" % seconds)

    def command(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.acks.pop(command, None)
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)


def send_velocity(link, vn, ve, vd):
    """Velocity-only command in the local NED frame (position, accel and yaw ignored)."""
    m, mav = link.m, link.mv.mavlink
    type_mask = 0b110111000111  # use vx, vy, vz only
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component, mav.MAV_FRAME_LOCAL_NED, type_mask,
        0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)


def set_param(link, name, value):
    m, mav = link.m, link.mv.mavlink
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mav.MAV_PARAM_TYPE_REAL32)
    if link.wait_until(lambda: name in link.params, 4):
        print(f"  param {name} = {link.params[name]}")
    else:
        print(f"  WARNING: no confirmation that {name} was set. Check it in the MAVProxy console: param show {name}")


def request_streams(link):
    """Ask for position data two ways, in case one is ignored."""
    mav = link.mv.mavlink
    link.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, LOCAL_POSITION_NED_ID, int(1e6 / 10.0))
    link.m.mav.request_data_stream_send(link.m.target_system, link.m.target_component,
                                        mav.MAV_DATA_STREAM_ALL, 10, 1)


def ensure_guided_and_armed(link, timeout=180):
    """Switch to GUIDED and arm, retrying until both are true.

    GUIDED is refused until GPS and the EKF have a position, and arming is refused until the
    pre-arm checks pass. That takes about a minute after SITL starts, so both are retried together.
    """
    mav = link.mv.mavlink
    t0 = time.time()
    while time.time() - t0 < timeout:
        link.command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_GUIDED)
        link.wait_until(lambda: link.custom_mode == COPTER_GUIDED, 3)
        if link.custom_mode != COPTER_GUIDED:
            print(f"  GUIDED not accepted yet (mode number {link.custom_mode}, reply "
                  f"{link.acks.get(mav.MAV_CMD_DO_SET_MODE)}). Waiting for GPS / EKF ...")
            link.pump(2)
            continue
        link.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if link.wait_until(lambda: link.armed, 4):
            print("armed, mode GUIDED")
            return
        print(f"  not armed yet (reply {link.acks.get(mav.MAV_CMD_COMPONENT_ARM_DISARM)}). Retrying ...")
    raise RuntimeError("could not reach GUIDED + armed within %d s. Read the [ArduPilot] messages above." % timeout)


def takeoff(link, alt, tries=4):
    """Send take-off and check the reply. If refused, show why, redo mode/arm and try again."""
    mav = link.mv.mavlink
    for _ in range(tries):
        link.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        link.wait_until(lambda: mav.MAV_CMD_NAV_TAKEOFF in link.acks, 4)
        res = link.acks.get(mav.MAV_CMD_NAV_TAKEOFF)
        if res == 0:
            print("take-off accepted")
            return
        print("  take-off REFUSED (reply %s)" % res if res is not None else "  no reply to the take-off command")
        ensure_guided_and_armed(link)
    raise RuntimeError("take-off was never accepted. Read the messages above.")


def wait_for_altitude(link, alt, timeout=60):
    t0 = last_print = time.time()
    while time.time() - t0 < timeout:
        link.poll(0.5)
        if link.pos is not None and -link.pos.z > 0.9 * alt:
            return
        if time.time() - last_print > 5:
            last_print = time.time()
            shown = "no position messages yet" if link.pos is None else f"{-link.pos.z:.1f} m"
            print(f"  waiting for altitude: {shown}, mode number {link.custom_mode}")
            if link.n_pos == 0:
                request_streams(link)
    raise RuntimeError(f"did not reach {alt:.0f} m in {timeout} s: position messages seen = {link.n_pos}, "
                       f"mode number = {link.custom_mode}, armed = {link.armed}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="udp:127.0.0.1:14550")
    ap.add_argument("--law", choices=LAW_NAMES, default="l1")
    ap.add_argument("--radius-factor", type=float, default=2.0,
                    help="planned radius / nominal r_min (2.0 needs ~1 m/s^2 sideways, near the ArduCopter default)")
    ap.add_argument("--speed", type=float, default=QUADCOPTER.airspeed, help="cruise speed, m/s")
    ap.add_argument("--alt", type=float, default=10.0, help="flight altitude, m")
    ap.add_argument("--horizon", type=float, default=0.5, help="s, acts like a gain (velocity-loop response time)")
    ap.add_argument("--rate", type=float, default=10.0, help="command rate, Hz")
    ap.add_argument("--set-param", action="append", default=[], metavar="NAME=VALUE",
                    help="ArduPilot parameter to set before flying; repeatable, e.g. WPNAV_ACCEL=300")
    ap.add_argument("--out", default="results/sitl_run.csv")
    args = ap.parse_args()

    from pymavlink import mavutil  # imported here so the rest of the repo works without it

    print("connecting to", args.connect, "...")
    m = mavutil.mavlink_connection(args.connect)
    link = Link(m, mavutil)
    link.wait_vehicle(30)
    print(f"connected to vehicle: system {link.veh_sys}, component {link.veh_comp}, "
          f"mode number {link.custom_mode}, armed {link.armed}")

    for item in args.set_param:
        name, value = item.split("=")
        set_param(link, name, float(value))

    request_streams(link)
    ensure_guided_and_armed(link)
    print("taking off to %.0f m ..." % args.alt)
    takeoff(link, args.alt)
    wait_for_altitude(link, args.alt)
    link.pump(2)

    # Build the path so it starts where the copter is now, heading east.
    x0, y0 = link.pos.y, link.pos.x                # (NED) east, north
    veh = QUADCOPTER
    rho = args.radius_factor * veh.r_min
    path = build_route(standard_route(veh.r_min), rho)
    path.x = path.x + x0
    path.y = path.y + y0
    law = make_law(args.law, veh.r_min)
    law.reset(path, x0, y0)
    print(f"flying {args.law}, planned radius {rho:.1f} m, path {path.length:.0f} m")

    dt = 1.0 / args.rate
    t_start = last_cmd = time.time()
    t_limit = 4.0 * path.length / args.speed
    rows, done = [], False
    try:
        while time.time() - t_start < t_limit:
            msg = link.poll(0.5)
            if msg is None or msg.get_type() != "LOCAL_POSITION_NED":
                continue
            if time.time() - last_cmd < 0.9 * dt:
                continue
            last_cmd = time.time()
            x_e, y_n = msg.y, msg.x                        # (NED) east, north
            v_e, v_n = msg.vy, msg.vx                      # (NED) east, north velocity
            vn, ve, a_cmd = velocity_command(law, path, x_e, y_n, v_e, v_n, args.speed, args.horizon)
            vd = 0.5 * (-args.alt - msg.z)                 # (NED) altitude hold, z is down
            send_velocity(link, vn, ve, vd)
            rows.append((time.time() - t_start, x_e, y_n, v_e, v_n, a_cmd, law.idx))
            if finished(law, path, x_e, y_n, 0.4 * veh.r_min):
                done = True
                break
    finally:
        link.command(link.mv.mavlink.MAV_CMD_DO_SET_MODE, link.mv.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     COPTER_LAND)
        print("landing")

    if len(rows) < 10:
        print("too few samples were recorded; nothing saved")
        return
    s = save_run(args.out, rows, path, veh.r_min, args.speed)
    print(f"finished={done}  rms error={s['rms_cte_m']:.2f} m ({s['rms_cte_over_rmin']:.3f} r_min)  "
          f"max error={s['max_cte_m']:.2f} m ({s['max_cte_over_rmin']:.3f} r_min)")
    print("saved", args.out, "and its path file")


if __name__ == "__main__":
    main()
