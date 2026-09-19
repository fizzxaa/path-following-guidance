"""Run sitl/sitl_quad_runner.py end to end against a FAKE MAVLink connection.

The fake copter is deliberately awkward, to mimic what went wrong on a real SITL:
  - the connection starts with target_system = 0 (the vehicle is not known yet)
  - ground-station heartbeats are mixed in with the vehicle's
  - commands not addressed to system 1 are ignored
  - GUIDED, arming and the first take-off are refused at first, with STATUSTEXT reasons
  - position data only arrives after it is requested
This checks the runner's own Python. It does NOT prove ArduPilot accepts the messages.
"""
import importlib.util
import os
import sys
import types

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(__file__))


class _Const:
    MAV_CMD_SET_MESSAGE_INTERVAL = 511
    MAV_CMD_NAV_TAKEOFF = 22
    MAV_CMD_DO_SET_MODE = 176
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_DATA_STREAM_ALL = 0
    MAV_FRAME_LOCAL_NED = 1
    MAV_PARAM_TYPE_REAL32 = 9
    MAV_AUTOPILOT_INVALID = 8
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1


class Msg:
    def __init__(self, mtype, src_sys=1, src_comp=1, **kw):
        self._t, self._s, self._c = mtype, src_sys, src_comp
        self.__dict__.update(kw)

    def get_type(self):
        return self._t

    def get_srcSystem(self):
        return self._s

    def get_srcComponent(self):
        return self._c


def _ack(command, result):
    return Msg("COMMAND_ACK", command=command, result=result)


def _text(t):
    return Msg("STATUSTEXT", text=t)


class FakeMavSender:
    def __init__(self, conn):
        self.c = conn

    def command_long_send(self, sysid, comp, command, confirmation, p1, p2, p3, p4, p5, p6, p7):
        c = self.c
        if sysid != 1:                       # not addressed to the vehicle: ignored
            c.ignored_commands += 1
            return
        if command == _Const.MAV_CMD_DO_SET_MODE:
            mode = int(p2)
            if mode == 4 and c.mode_refusals_left > 0:
                c.mode_refusals_left -= 1
                c.queue += [_text("Mode change failed: requires position"), _ack(command, 4)]
            else:
                c.custom_mode = mode
                c.queue.append(_ack(command, 0))
        elif command == _Const.MAV_CMD_COMPONENT_ARM_DISARM:
            c.arm_calls += 1
            if p1 == 1 and c.arm_refusals_left > 0:
                c.arm_refusals_left -= 1
                c.queue += [_text("PreArm: waiting for navigation checks"), _ack(command, 4)]
            else:
                c.armed = (p1 == 1)
                c.queue.append(_ack(command, 0))
        elif command == _Const.MAV_CMD_NAV_TAKEOFF:
            c.takeoff_calls += 1
            good = c.armed and c.custom_mode == 4
            if good and c.takeoff_refusals_left > 0:
                c.takeoff_refusals_left -= 1
                good = False
            if good:
                c.target_alt, c.climbing = p7, True
                c.queue.append(_ack(command, 0))
            else:
                c.queue += [_text("Takeoff refused"), _ack(command, 4)]
        elif command == _Const.MAV_CMD_SET_MESSAGE_INTERVAL:
            c.intervals[int(p1)] = p2
            if not c.ignore_interval:
                c.streams_on = True

    def param_set_send(self, sysid, comp, name, value, ptype):
        if sysid != 1:
            self.c.ignored_commands += 1
            return
        self.c.params[name.decode()] = value
        self.c.queue.append(Msg("PARAM_VALUE", param_id=name.decode(), param_value=value))

    def request_data_stream_send(self, sysid, comp, stream, rate, start):
        if sysid == 1:
            self.c.streams_on = True

    def set_position_target_local_ned_send(self, t, sysid, comp, frame, mask,
                                           x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate):
        assert frame == _Const.MAV_FRAME_LOCAL_NED
        assert mask == 0b110111000111
        if sysid != 1:
            self.c.ignored_commands += 1
            return
        self.c.cmd = (vx, vy, vz)
        self.c.n_commands += 1


class FakeConn:
    def __init__(self, clock, ignore_interval=False, only_gcs=False):
        self.clock = clock
        self.target_system, self.target_component = 0, 0
        self.mav = FakeMavSender(self)
        self.ignore_interval, self.only_gcs = ignore_interval, only_gcs
        self.mode_refusals_left, self.arm_refusals_left, self.takeoff_refusals_left = 2, 1, 1
        self.params, self.intervals, self.queue = {}, {}, []
        self.streams_on = False
        self.custom_mode, self.armed = 0, False
        self.arm_calls = self.takeoff_calls = self.n_commands = self.ignored_commands = 0
        self.n = self.e = self.d = 0.0
        self.vn = self.ve = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.target_alt, self.climbing = 0.0, False
        self.last_t = clock[0]
        self.next_hb = self.next_gcs = self.next_pos = 0.0

    def _advance(self):
        dt_total = self.clock[0] - self.last_t
        self.last_t = self.clock[0]
        steps = max(1, int(dt_total / 0.05))
        dt = dt_total / steps if dt_total > 0 else 0.0
        for _ in range(steps):
            if self.custom_mode == 4 and self.armed:
                if self.climbing and -self.d < self.target_alt - 0.05:
                    self.d -= 2.0 * dt
                elif self.cmd != (0.0, 0.0, 0.0):
                    self.climbing = False
                    for attr, tgt in (("vn", self.cmd[0]), ("ve", self.cmd[1])):
                        v = getattr(self, attr)
                        a = max(-2.0, min(2.0, (tgt - v) / 0.5))
                        setattr(self, attr, v + a * dt)
                    self.d += self.cmd[2] * dt
                    self.n += self.vn * dt
                    self.e += self.ve * dt
            elif self.custom_mode == 9:
                self.vn = self.ve = 0.0
                self.d = min(0.0, self.d + 1.0 * dt)

    def _due(self):
        now = self.clock[0]
        if self.queue:
            return self.queue.pop(0)
        if now >= self.next_gcs:
            self.next_gcs += 1.0
            return Msg("HEARTBEAT", src_sys=255, src_comp=190, type=6, autopilot=8, base_mode=0, custom_mode=0)
        if now >= self.next_hb:
            self.next_hb += 1.0
            if self.only_gcs:
                return None
            return Msg("HEARTBEAT", type=2, autopilot=3, custom_mode=self.custom_mode,
                       base_mode=1 | (128 if self.armed else 0))
        if self.streams_on and now >= self.next_pos:
            self.next_pos += 0.1
            return Msg("LOCAL_POSITION_NED", x=self.n, y=self.e, z=self.d, vx=self.vn, vy=self.ve, vz=0.0)
        return None

    def _next_event(self):
        ev = [self.next_gcs, self.next_hb]
        if self.streams_on:
            ev.append(self.next_pos)
        return min(ev)

    def recv_match(self, type=None, blocking=False, timeout=None):
        deadline = self.clock[0] + (timeout if timeout is not None else (5.0 if blocking else 0.0))
        while True:
            self._advance()
            msg = self._due()
            if msg is not None:
                if type is None or msg.get_type() == type:
                    return msg
                continue                                  # filtered out, like pymavlink
            if self.clock[0] >= deadline:
                return None
            self.clock[0] = min(deadline, max(self.clock[0], self._next_event()))


def _load_runner(monkeypatch, conn, clock):
    mavutil = types.SimpleNamespace(mavlink=_Const, mavlink_connection=lambda *a, **k: conn)
    pymavlink = types.ModuleType("pymavlink")
    pymavlink.mavutil = mavutil
    monkeypatch.setitem(sys.modules, "pymavlink", pymavlink)
    monkeypatch.setitem(sys.modules, "pymavlink.mavutil", mavutil)
    import time as _time
    monkeypatch.setattr(_time, "time", lambda: clock[0])
    monkeypatch.setattr(_time, "sleep", lambda s: clock.__setitem__(0, clock[0] + s))
    spec = importlib.util.spec_from_file_location("sitl_quad_runner",
                                                  os.path.join(ROOT, "sitl", "sitl_quad_runner.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(params=[False, True], ids=["interval-ok", "interval-ignored"])
def runner(monkeypatch, request):
    clock = [0.0]
    conn = FakeConn(clock, ignore_interval=request.param)
    return _load_runner(monkeypatch, conn, clock), conn


@pytest.mark.parametrize("law", ["pure_pursuit", "l1", "vector_field", "lead_vf"])
def test_runner_flies_the_route_against_an_awkward_fake_copter(runner, monkeypatch, tmp_path, capsys, law):
    mod, conn = runner
    out = tmp_path / f"{law}.csv"
    monkeypatch.setattr(sys, "argv", ["x", "--law", law, "--radius-factor", "2.0",
                                      "--set-param", "WPNAV_ACCEL=300", "--out", str(out)])
    mod.main()
    text = capsys.readouterr().out
    assert "connected to vehicle: system 1" in text               # found the vehicle, not system 0
    assert conn.ignored_commands == 0                             # every command was addressed to system 1
    assert conn.armed and conn.arm_calls >= 2                     # retried arming
    assert conn.mode_refusals_left == 0                           # GUIDED retried until accepted
    assert conn.takeoff_calls >= 2                                # first take-off refused, then retried
    assert "[ArduPilot] Mode change failed" in text               # the reasons were shown, not lost
    assert "[ArduPilot] PreArm" in text
    assert conn.params.get("WPNAV_ACCEL") == 300.0
    assert conn.streams_on
    assert conn.custom_mode == 9                                  # ends by landing
    assert conn.n_commands > 200
    d = np.genfromtxt(out, delimiter=",", names=True)
    assert len(d) > 200
    assert np.sqrt(np.mean(d["cte_m"] ** 2)) < 3.0
    assert (tmp_path / f"{law}_path.csv").exists()


def test_runner_gives_a_clear_error_if_no_vehicle_is_heard(monkeypatch, tmp_path):
    clock = [0.0]
    conn = FakeConn(clock, only_gcs=True)
    mod = _load_runner(monkeypatch, conn, clock)
    monkeypatch.setattr(sys, "argv", ["x", "--out", str(tmp_path / "x.csv")])
    with pytest.raises(RuntimeError, match="no heartbeat from a vehicle"):
        mod.main()
