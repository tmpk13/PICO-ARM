"""Onshape URDF -> SKR Pico IK sidecar.

Reads a joystick at /dev/input/jsN, runs ikpy on a URDF tree to map
end-effector Cartesian targets to joint angles, and streams jog
commands to one or more SKR Picos over USB CDC -- exactly like the
Rust host, but with the joint-space substituted for stick-driven
Cartesian-space input.

  pixi run python ik_host.py                  # uses ../config-ik.toml
  pixi run python ik_host.py path/to/cfg.toml

The arm is assumed to be parked at its home pose when this starts, and
jog rates are integrated forward from there. Set the home pose in the
config: `cartesian.home_joints` (joint name -> degrees) pins each joint
directly and is preferred; `cartesian.home_xyz` is the legacy fallback
that derives the pose via one-time IK. Control is velocity-level, so at
zero stick deflection every jog is 0 -- the arm does not move at launch.

With AS5600 feedback, set `sensors.zero_at_home = true` to recapture the
encoder offsets at launch so the current reading maps to home: wherever
the arm physically sits when the host starts is declared home, and the
feedback agrees with the assumed pose instead of snapping to it.

Stops cleanly on Ctrl-C: sends `jog * 0` and `disable all` to every
board before exiting.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import select
import signal
import struct
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import serial
import tomli
from ikpy.chain import Chain

HOST_VERSION = "0.5.0"  # must match firmware/host Cargo.toml
JS_EVENT_SIZE = 8
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_MAX = 32767.0
AXIS_TOKENS = {"x", "y", "z", "e"}

# AS5600 encoder feedback (firmware-sensors CSV stream) is considered valid
# only if a frame arrived within this many seconds; otherwise the affected
# joints fall back to dead reckoning.
SENSOR_STALE_SEC = 0.5

# -----------------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------------


@dataclass
class JointMap:
    target: str
    joint: str
    steps_per_rad: float
    invert: bool = False
    accel: int = 0
    microsteps: int = 0
    run_current_ma: int = 800
    hold_current_ma: int = 400
    hold_delay: int = 8
    spreadcycle: bool = False
    interpolate: bool = True


@dataclass
class PassthroughAxis:
    """Joystick axis -> stepper jog, identical to the Rust host. Used for
    end-effector DOFs that aren't part of the IK chain (e.g. a wrist or
    gripper served by the same board)."""

    index: int
    target: str
    sensitivity: float
    deadzone: float = 0.0
    invert: bool = False
    accel: int = 0
    microsteps: int = 0
    run_current_ma: int = 800
    hold_current_ma: int = 400
    hold_delay: int = 8
    spreadcycle: bool = False
    interpolate: bool = True


@dataclass
class FanMap:
    index: int
    target: int
    mode: str = "toggle"


@dataclass
class EncoderMap:
    """Binds one AS5600 channel from the firmware-sensors CSV stream
    (`a0,a1,a2,a3`, degrees) to a URDF joint. The raw 0..360 reading is
    converted to a joint angle in radians as:

        joint_deg = (raw_deg - offset_deg) / scale   (then optionally negated)

    wrapped to (-180, 180]. `offset_deg` is the raw reading when the joint
    is at its zero; tune it until the host's reported angle matches the real
    joint. `scale` is encoder-degrees per joint-degree (gear ratio; 1.0 for a
    sensor on the joint axis). `invert` flips the sense to match the URDF axis.
    """

    channel: int
    joint: str
    offset_deg: float = 0.0
    invert: bool = False
    scale: float = 1.0


@dataclass
class SensorCfg:
    serial: str
    encoders: list["EncoderMap"] = field(default_factory=list)
    enabled: bool = True
    # When true, each encoder's offset_deg is recaptured at startup so the
    # reading taken the moment the host launches maps to the home joint
    # angle. In other words, wherever the arm physically sits at launch is
    # declared "home" -- the manual offset_deg values are ignored. Leave
    # false to use the hand-calibrated offset_deg instead.
    zero_at_home: bool = False


@dataclass
class BoardConfig:
    serial: str
    name: str | None = None
    joints: list[JointMap] = field(default_factory=list)
    axes: list[PassthroughAxis] = field(default_factory=list)
    fans: list[FanMap] = field(default_factory=list)


@dataclass
class CartesianCfg:
    range_xyz: tuple[float, float, float]
    axis_x: int
    axis_y: int
    axis_z: int
    deadzone: float = 0.0
    invert_x: bool = False
    invert_y: bool = False
    invert_z: bool = False
    # Max end-effector linear speed at full stick deflection, per axis
    # (m/s). Stick deflection maps linearly to commanded EE velocity.
    # Defaults to range_xyz when absent so old configs keep a sane scale.
    max_speed_xyz: tuple[float, float, float] = (0.1, 0.1, 0.1)
    # Damped-least-squares damping factor lambda. Larger = more robust
    # near singularities but more tracking error; smaller = crisper but
    # twitchy at workspace edges.
    dls_lambda: float = 0.05
    # Assumed joint configuration at startup: { joint_name -> radians }.
    # When present this is the home pose -- the host seeds its dead-reckoned
    # joint state directly from it (no IK solve), so the arm assumes it is
    # parked exactly here and commands zero motion until a stick deflects.
    # Preferred over home_xyz, which only fixes the EE position and leaves
    # the held joints wherever IK happens to land.
    home_joints: dict[str, float] | None = None
    # Legacy/optional: EE position (m) the arm is assumed parked at. Used
    # to seed the joint state via one-time IK only when home_joints is
    # absent. May be None when home_joints is supplied.
    home_xyz: tuple[float, float, float] | None = None


@dataclass
class Config:
    device: str
    urdf: str
    chain_elements: list[str]
    chain_base_type: str
    active_joints: list[str]
    cartesian: CartesianCfg
    boards: list[BoardConfig]
    poll_hz: int = 40
    log: bool = False
    enable_on_start: bool = True
    safe_stop_on_exit: bool = True
    sensors: SensorCfg | None = None


def _opt(d: dict, key: str, default: Any) -> Any:
    v = d.get(key)
    return default if v is None else v


def load_config(path: Path) -> Config:
    with open(path, "rb") as f:
        raw = tomli.load(f)

    cart_raw = raw.get("cartesian")
    if cart_raw is None:
        raise ValueError("config missing [cartesian] block")
    range_xyz = tuple(cart_raw["range_xyz"])
    ms = _opt(cart_raw, "max_speed_xyz", range_xyz)
    max_speed = (float(ms[0]), float(ms[1]), float(ms[2]))
    # Home pose. home_joints (joint name -> degrees) is preferred and is
    # converted to radians here; home_xyz stays optional for back-compat.
    hj_raw = cart_raw.get("home_joints")
    home_joints = None
    if hj_raw is not None:
        home_joints = {str(k): math.radians(float(v)) for k, v in hj_raw.items()}
    hx_raw = cart_raw.get("home_xyz")
    home_xyz = ((float(hx_raw[0]), float(hx_raw[1]), float(hx_raw[2]))
                if hx_raw is not None else None)
    if home_joints is None and home_xyz is None:
        raise ValueError(
            "[cartesian] needs home_joints (joint->deg) or home_xyz")
    cart = CartesianCfg(
        range_xyz=range_xyz,
        axis_x=int(cart_raw["axis_x"]),
        axis_y=int(cart_raw["axis_y"]),
        axis_z=int(cart_raw["axis_z"]),
        deadzone=float(_opt(cart_raw, "deadzone", 0.0)),
        invert_x=bool(_opt(cart_raw, "invert_x", False)),
        invert_y=bool(_opt(cart_raw, "invert_y", False)),
        invert_z=bool(_opt(cart_raw, "invert_z", False)),
        max_speed_xyz=max_speed,
        dls_lambda=float(_opt(cart_raw, "dls_lambda", 0.05)),
        home_joints=home_joints,
        home_xyz=home_xyz,
    )

    boards = []
    for b in raw.get("boards", []):
        joints = [
            JointMap(
                target=j["target"],
                joint=j["joint"],
                steps_per_rad=float(j["steps_per_rad"]),
                invert=bool(_opt(j, "invert", False)),
                accel=int(_opt(j, "accel", 0)),
                microsteps=int(_opt(j, "microsteps", 0)),
                run_current_ma=int(_opt(j, "run_current_ma", 800)),
                hold_current_ma=int(_opt(j, "hold_current_ma", 400)),
                hold_delay=int(_opt(j, "hold_delay", 8)),
                spreadcycle=bool(_opt(j, "spreadcycle", False)),
                interpolate=bool(_opt(j, "interpolate", True)),
            )
            for j in b.get("joints", [])
        ]
        axes = [
            PassthroughAxis(
                index=int(a["index"]),
                target=a["target"],
                sensitivity=float(a["sensitivity"]),
                deadzone=float(_opt(a, "deadzone", 0.0)),
                invert=bool(_opt(a, "invert", False)),
                accel=int(_opt(a, "accel", 0)),
                microsteps=int(_opt(a, "microsteps", 0)),
                run_current_ma=int(_opt(a, "run_current_ma", 800)),
                hold_current_ma=int(_opt(a, "hold_current_ma", 400)),
                hold_delay=int(_opt(a, "hold_delay", 8)),
                spreadcycle=bool(_opt(a, "spreadcycle", False)),
                interpolate=bool(_opt(a, "interpolate", True)),
            )
            for a in b.get("axes", [])
        ]
        fans = [
            FanMap(
                index=int(f["index"]),
                target=int(f["target"]),
                mode=str(_opt(f, "mode", "toggle")),
            )
            for f in b.get("fans", [])
        ]
        boards.append(
            BoardConfig(
                serial=b["serial"],
                name=_opt(b, "name", None),
                joints=joints,
                axes=axes,
                fans=fans,
            )
        )
    if not boards:
        raise ValueError("config has no [[boards]] entries")

    sensors = None
    sraw = raw.get("sensors")
    if sraw is not None:
        encoders = [
            EncoderMap(
                channel=int(e["channel"]),
                joint=e["joint"],
                offset_deg=float(_opt(e, "offset_deg", 0.0)),
                invert=bool(_opt(e, "invert", False)),
                scale=float(_opt(e, "scale", 1.0)),
            )
            for e in sraw.get("encoders", [])
        ]
        sensors = SensorCfg(
            serial=sraw["serial"],
            encoders=encoders,
            enabled=bool(_opt(sraw, "enabled", True)),
            zero_at_home=bool(_opt(sraw, "zero_at_home", False)),
        )

    return Config(
        device=raw["device"],
        urdf=raw["urdf"],
        chain_elements=list(raw["chain_elements"]),
        chain_base_type=str(_opt(raw, "chain_base_type", "link")),
        active_joints=list(raw["active_joints"]),
        cartesian=cart,
        boards=boards,
        poll_hz=int(_opt(raw, "poll_hz", 40)),
        log=bool(_opt(raw, "log", False)),
        enable_on_start=bool(_opt(raw, "enable_on_start", True)),
        safe_stop_on_exit=bool(_opt(raw, "safe_stop_on_exit", True)),
        sensors=sensors,
    )


# -----------------------------------------------------------------------------
# AS5600 encoder math
# -----------------------------------------------------------------------------


def encoder_rad(raw_deg: float, enc: EncoderMap) -> float:
    """Raw AS5600 reading (degrees, 0..360) -> joint angle in radians.

    Applies the configured zero offset, gear scale, and sense inversion, then
    wraps to (-180, 180] before converting to radians.
    """
    a = raw_deg - enc.offset_deg
    if enc.scale:
        a /= enc.scale
    if enc.invert:
        a = -a
    a = ((a + 180.0) % 360.0) - 180.0
    return math.radians(a)


def offset_for_home(raw_deg: float, home_rad: float, enc: EncoderMap) -> float:
    """Solve for the offset_deg that makes encoder_rad(raw_deg, enc) equal
    home_rad. Used by zero-at-home: the reading captured at launch is
    declared to correspond to the home joint angle, so the encoder feedback
    agrees with the assumed start pose and can't yank the arm on tick 1."""
    home_deg = math.degrees(home_rad)
    pre = -home_deg if enc.invert else home_deg
    return raw_deg - enc.scale * pre


# -----------------------------------------------------------------------------
# URDF chain + IK wrapper
# -----------------------------------------------------------------------------


_CONTINUOUS_LIMIT = 2 * math.pi


def _patch_continuous_joints(src: Path) -> Path:
    """Onshape emits `<joint type="continuous">` for unlimited revolutes;
    ikpy only knows about revolute/prismatic/fixed. Convert them in
    place to revolute with a +/-2pi limit and write to a temp file."""
    tree = ET.parse(src)
    root = tree.getroot()
    patched = False
    for joint in root.findall("joint"):
        if joint.get("type") == "continuous":
            joint.set("type", "revolute")
            limit = joint.find("limit")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.set("lower", str(-_CONTINUOUS_LIMIT))
            limit.set("upper", str(_CONTINUOUS_LIMIT))
            if limit.get("effort") is None:
                limit.set("effort", "1")
            if limit.get("velocity") is None:
                limit.set("velocity", "1")
            patched = True
    if not patched:
        return src
    fd, out = tempfile.mkstemp(prefix="urdf_patched_", suffix=".urdf")
    os.close(fd)
    tree.write(out, xml_declaration=True, encoding="utf-8")
    return Path(out)


class IkSolver:
    """Wraps an ikpy Chain plus the joint-name -> chain-index lookup.

    Only the driven joints (the ones listed in active_joints / bound to
    steppers) are IK degrees of freedom. ikpy auto-extends the chain past
    the last listed element to the tree leaf, so non-driven moving joints
    (e.g. the wrist pivot revolute_1) can still appear in the chain. We
    build the active_links_mask from the driven set alone, so every other
    joint -- fixed or merely undriven -- is held at its seed value and the
    solver never moves it.
    """

    def __init__(self, urdf_path: Path, elements: list[str], base_type: str,
                 driven_joints: list[str]):
        # ikpy refuses `type="continuous"` (the type Onshape emits for
        # joints with no <limit>). Rewrite continuous->revolute with a
        # generous +/-2*pi limit so the optimizer's bound-aware path
        # still works.
        patched_urdf = _patch_continuous_joints(urdf_path)
        # Build the chain once to discover its links, then rebuild with an
        # active mask that flags ONLY the driven joints. Holding every
        # other joint (including undriven moving ones like the wrist pivot)
        # keeps the IK exactly N-DOF for N driven joints.
        driven_set = set(driven_joints)
        first = Chain.from_urdf_file(
            str(patched_urdf),
            base_elements=elements,
            base_element_type=base_type,
        )
        mask = [link.name in driven_set for link in first.links]
        chain = Chain.from_urdf_file(
            str(patched_urdf),
            base_elements=elements,
            base_element_type=base_type,
            active_links_mask=mask,
        )
        self.chain = chain
        # Map driven joint name -> index in chain.links / IK output.
        self.joint_index: dict[str, int] = {}
        for j in driven_joints:
            idx = None
            for i, link in enumerate(chain.links):
                if link.name == j:
                    idx = i
                    break
            if idx is None:
                names = ", ".join(link.name for link in chain.links)
                raise ValueError(
                    f"joint {j!r} not in URDF chain (chain links: {names})"
                )
            self.joint_index[j] = idx
        # Seed at each joint's bounds midpoint -- 0 is outside the
        # bounds for some Onshape-exported joints (e.g. linkage_1
        # whose limit is 0.349..1.920) so scipy.least_squares would
        # reject it.
        seed = np.zeros(len(chain.links))
        for i, link in enumerate(chain.links):
            if not mask[i]:
                continue
            lo, hi = link.bounds
            if math.isfinite(lo) and math.isfinite(hi):
                seed[i] = 0.5 * (lo + hi)
        self._seed = seed

    def solve(self, target_xyz: tuple[float, float, float]) -> np.ndarray:
        """Return full IK solution vector (len == len(chain.links))."""
        sol = self.chain.inverse_kinematics(
            target_position=list(target_xyz),
            initial_position=self._seed,
        )
        self._seed = sol
        return sol

    def joint_angle(self, sol: np.ndarray, joint: str) -> float:
        return float(sol[self.joint_index[joint]])

    def seed_vector(self) -> np.ndarray:
        """Full-length joint vector seeded at each joint's bounds midpoint
        (fixed links stay 0). Use as the starting point when building a
        home pose directly in joint space."""
        return self._seed.copy()

    def link_index(self, name: str) -> int:
        """Chain index of a link/joint by name. Unlike joint_index this
        also resolves held (non-driven) joints such as the wrist pivot,
        so a home pose can pin them too."""
        for i, link in enumerate(self.chain.links):
            if link.name == name:
                return i
        names = ", ".join(link.name for link in self.chain.links)
        raise ValueError(f"joint {name!r} not in URDF chain (chain: {names})")

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        """End-effector position (xyz) for a full joint vector q."""
        return np.asarray(self.chain.forward_kinematics(q))[:3, 3]

    def position_jacobian(self, q: np.ndarray, active: list[int],
                          eps: float = 1e-6) -> np.ndarray:
        """3 x len(active) finite-difference Jacobian d(ee_xyz)/dq for the
        driven joints. ikpy exposes no analytic Jacobian, but FK is a few
        4x4 mults so perturbing each driven joint once per tick is cheap."""
        p0 = self.ee_position(q)
        jac = np.zeros((3, len(active)))
        for k, idx in enumerate(active):
            qp = q.copy()
            qp[idx] += eps
            jac[:, k] = (self.ee_position(qp) - p0) / eps
        return jac


# -----------------------------------------------------------------------------
# Joystick reader
# -----------------------------------------------------------------------------


class JoystickReader(threading.Thread):
    """Background thread: keeps /dev/input/jsN open, mirrors axis values
    into self.axes, fires on_button(button_idx, pressed) on edges.
    Returns the snapshot through .axes_snapshot()."""

    def __init__(self, device: str, on_button):
        super().__init__(name="js-reader", daemon=True)
        self.device = device
        self.on_button = on_button
        self._lock = threading.Lock()
        self._axes: list[int] = []
        # Per-axis rest value, learned from the synthetic JS_EVENT_INIT
        # events the kernel emits when the device is opened. Triggers
        # (and any off-center stick) report their rest value here, and we
        # subtract it so "untouched" is always 0 -- otherwise a trigger
        # resting at -32768 reads as full deflection and jogs an axis the
        # moment the host starts.
        self._rest: list[int] = []
        self.connected = False
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def axes_snapshot(self) -> list[int]:
        with self._lock:
            out: list[int] = []
            for i, a in enumerate(self._axes):
                r = self._rest[i] if i < len(self._rest) else 0
                v = a - r
                if v > JS_MAX:
                    v = int(JS_MAX)
                elif v < -JS_MAX:
                    v = -int(JS_MAX)
                out.append(v)
            return out

    def _zero_axes(self):
        with self._lock:
            for i in range(len(self._axes)):
                self._axes[i] = 0
            # Drop the learned rest bias; it is relearned from the INIT
            # events of the next connection.
            self._rest = []

    def run(self):
        warned = False
        while not self._stop.is_set():
            try:
                fd = os.open(self.device, os.O_RDONLY)
            except OSError as e:
                if not warned:
                    print(f"joystick: cannot open {self.device} ({e}); retrying",
                          file=sys.stderr)
                    warned = True
                time.sleep(1.0)
                continue
            print(f"joystick: connected {self.device}", file=sys.stderr)
            self.connected = True
            self._zero_axes()
            warned = False
            try:
                self._read_loop(fd)
            finally:
                os.close(fd)
                self.connected = False
                self._zero_axes()
                print("joystick: disconnected, will retry", file=sys.stderr)
                time.sleep(0.3)

    def _read_loop(self, fd: int):
        # poll() so we can wake up on stop without blocking forever.
        poller = select.poll()
        poller.register(fd, select.POLLIN)
        button_state: dict[int, int] = {}
        while not self._stop.is_set():
            evs = poller.poll(200)
            if not evs:
                continue
            try:
                buf = os.read(fd, JS_EVENT_SIZE)
            except OSError:
                return
            if len(buf) != JS_EVENT_SIZE:
                return
            _t, value, ev_type, number = struct.unpack("<IhBB", buf)
            is_init = bool(ev_type & JS_EVENT_INIT)
            kind = ev_type & ~JS_EVENT_INIT
            if kind == JS_EVENT_AXIS:
                with self._lock:
                    if number >= len(self._axes):
                        self._axes.extend([0] * (number + 1 - len(self._axes)))
                    self._axes[number] = value
                    # The kernel replays each axis once with JS_EVENT_INIT
                    # set right after open; that value is the rest pose.
                    if is_init:
                        if number >= len(self._rest):
                            self._rest.extend(
                                [0] * (number + 1 - len(self._rest)))
                        self._rest[number] = value
            elif kind == JS_EVENT_BUTTON:
                prev = button_state.get(number, 0)
                button_state[number] = value
                if not is_init:
                    if value != 0 and prev == 0:
                        self.on_button(number, True)
                    elif value == 0 and prev != 0:
                        self.on_button(number, False)


# -----------------------------------------------------------------------------
# AS5600 encoder reader (firmware-sensors CSV stream)
# -----------------------------------------------------------------------------


class SensorReader(threading.Thread):
    """Background thread that reads the firmware-sensors CSV stream from its
    own USB-CDC serial port and exposes the latest raw AS5600 angles.

    The firmware emits one line per frame, `a0,a1,a2,a3\\r\\n`, where each
    field is an angle in degrees (or `NaN` for a bus that has never read).
    It only streams -- it does not answer the `version` handshake -- so this
    is a separate port from the stepper boards. Reconnects on failure.
    """

    def __init__(self, port_path: str, n_channels: int = 4):
        super().__init__(name="sensor-reader", daemon=True)
        self.port_path = port_path
        self.n = n_channels
        self._lock = threading.Lock()
        self._raw: list[float] = [math.nan] * n_channels
        self._stamp = 0.0  # monotonic time of the last good frame
        self.connected = False
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def snapshot(self) -> tuple[list[float], float]:
        """Return (raw angles in degrees, monotonic timestamp of last frame)."""
        with self._lock:
            return list(self._raw), self._stamp

    def run(self):
        warned = False
        while not self._stop.is_set():
            try:
                port = serial.Serial(self.port_path, 115200, timeout=0.5)
            except Exception as e:
                if not warned:
                    print(f"sensors: cannot open {self.port_path} ({e}); "
                          f"retrying", file=sys.stderr)
                    warned = True
                time.sleep(1.0)
                continue
            print(f"sensors: connected {self.port_path}", file=sys.stderr)
            self.connected = True
            warned = False
            try:
                self._read_loop(port)
            except Exception as e:
                print(f"sensors: read error: {e}", file=sys.stderr)
            finally:
                try:
                    port.close()
                except Exception:
                    pass
                self.connected = False
                with self._lock:
                    self._raw = [math.nan] * self.n
                print("sensors: disconnected, will retry", file=sys.stderr)
                time.sleep(0.3)

    def _read_loop(self, port: serial.Serial):
        while not self._stop.is_set():
            line = port.readline()
            if not line:
                continue  # read timeout; loop back to check the stop flag
            text = line.decode("ascii", errors="ignore").strip()
            if not text:
                continue
            parts = text.split(",")
            if len(parts) < self.n:
                continue
            vals: list[float] = []
            for p in parts[: self.n]:
                try:
                    vals.append(float(p.strip()))  # float("NaN") is valid
                except ValueError:
                    vals.append(math.nan)
            with self._lock:
                self._raw = vals
                self._stamp = time.monotonic()


# -----------------------------------------------------------------------------
# Serial: writer + drainer threads per board. Same shape as Rust host.
# -----------------------------------------------------------------------------


@dataclass
class Board:
    label: str
    port: serial.Serial
    cfg: BoardConfig
    write_lock: threading.Lock = field(default_factory=threading.Lock)


def open_port(path: str) -> serial.Serial:
    return serial.Serial(path, 115200, timeout=0.2)


def parse_version(s: str) -> str | None:
    for tok in re.split(r"[^A-Za-z0-9.]", s):
        if tok.startswith("v"):
            rest = tok[1:]
            parts = rest.split(".")
            if len(parts) == 3 and all(p.isdigit() for p in parts):
                return rest
    return None


def handshake_version(port: serial.Serial) -> str:
    port.reset_input_buffer()
    port.write(b"\r\nversion\r\n")
    port.flush()
    deadline = time.monotonic() + 0.8
    accum = bytearray()
    while time.monotonic() < deadline:
        n = port.in_waiting
        if n:
            accum += port.read(n)
            if len(accum) > 4096:
                break
        else:
            time.sleep(0.02)
    text = accum.decode("ascii", errors="replace")
    v = parse_version(text)
    if v is None:
        raise RuntimeError(
            f"firmware did not return a parseable version (got: {text[:160]!r})"
        )
    return v


def spawn_drainer(board: Board):
    def drain():
        while True:
            try:
                n = board.port.in_waiting
                if n:
                    board.port.read(n)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)
    t = threading.Thread(target=drain, name=f"drainer:{board.label}", daemon=True)
    t.start()


def send(board: Board, cmd: str, log: bool):
    if log:
        print(f"[{board.label}] > {cmd.rstrip()}")
    with board.write_lock:
        try:
            board.port.write(cmd.encode("ascii"))
            board.port.flush()
        except Exception as e:
            print(f"[{board.label}] write failed: {e}", file=sys.stderr)


# -----------------------------------------------------------------------------
# Integrator: IK + jog dispatch at poll_hz.
# -----------------------------------------------------------------------------


def _stick(axes: list[int], idx: int, dz: float, invert: bool) -> float:
    if idx < 0 or idx >= len(axes):
        return 0.0
    raw = axes[idx] / JS_MAX
    if invert:
        raw = -raw
    mag = abs(raw)
    if mag < dz:
        return 0.0
    sign = 1.0 if raw > 0 else -1.0
    return (mag - dz) / (1.0 - dz) * sign


def _bucket_hz(v: float) -> int:
    """Round to nearest 25 steps/sec, like the Rust host. Same step
    quantization keeps firmware-side rate matching identical between
    the two front ends."""
    if abs(v) < 1.0:
        return 0
    s = 1 if v > 0 else -1
    m = max(1, round(abs(v) / 25.0)) * 25
    return s * m


def integrate_loop(cfg: Config, ik: IkSolver, boards: list[Board],
                   js: JoystickReader, stop_flag: threading.Event,
                   sensor: SensorReader | None = None):
    """Velocity-level resolved-rate control.

    Stick deflection -> commanded end-effector linear velocity v (m/s).
    Damped least squares maps v through the position Jacobian to joint
    rates: q_dot = J^T (J J^T + lambda^2 I)^-1 v. Those rates become
    `jog <target> <hz>` directly. At zero deflection q_dot = 0 and every
    jog is 0 -- there is no pose setpoint to chase, so the arm holds
    still and small Cartesian moves can't trigger null-space jumps the
    way per-tick pose IK on a redundant chain did.

    Joint positions are dead-reckoned by integrating the rates we send,
    purely to give the Jacobian an evaluation point next tick. When an
    AS5600 SensorReader is supplied, each tick first overwrites the driven
    joints with their measured angles so the Jacobian is evaluated at the
    real pose -- closing the loop and removing the home_xyz assumption for
    any joint with a live encoder. Channels that read NaN or go stale fall
    back to dead reckoning.
    """
    period = 1.0 / max(10, min(200, cfg.poll_hz))
    dt = period
    lam = max(1e-6, cfg.cartesian.dls_lambda)
    vmax = cfg.cartesian.max_speed_xyz

    # Starting joint configuration -- the arm is assumed to be parked
    # exactly here at launch, and we dead-reckon forward from it. Prefer an
    # explicit joint-space home (cartesian.home_joints): no IK solve, every
    # joint pinned where we say it is. Fall back to one-time IK on home_xyz
    # for back-compat. Either way the first tick commands zero motion (the
    # resolved-rate law sends 0 at zero stick deflection), so nothing moves
    # out of the gate.
    if cfg.cartesian.home_joints is not None:
        q = ik.seed_vector()
        for jname, ang in cfg.cartesian.home_joints.items():
            q[ik.link_index(jname)] = ang
        print(f"ik: home_joints -> q_home(active)="
              f"{[round(ik.joint_angle(q, j), 4) for j in ik.joint_index]}",
              file=sys.stderr)
    else:
        home_xyz = cfg.cartesian.home_xyz
        assert home_xyz is not None  # load_config requires one or the other
        q = ik.solve(home_xyz).copy()
        print(f"ik: home_xyz={home_xyz} -> q_home(active)="
              f"{[round(ik.joint_angle(q, j), 4) for j in ik.joint_index]}",
              file=sys.stderr)

    # Driven-joint chain indices, in the order the columns of the
    # Jacobian / q_dot are produced. col_of maps a URDF joint name to
    # its column so each [[boards.joints]] entry can find its rate.
    active = list(ik.joint_index.values())
    col_of = {name: k for k, name in enumerate(ik.joint_index)}
    bounds = [ik.chain.links[i].bounds for i in active]

    # Per board: { stepper_target -> last_hz (int) } for jog dedup.
    last_hz: list[dict[str, int]] = [{} for _ in boards]
    for bi, board in enumerate(boards):
        for jm in board.cfg.joints:
            last_hz[bi][jm.target] = 0

    # Passthrough axis bookkeeping (same logic as the Rust host).
    last_passthrough: list[dict[str, tuple[int, int]]] = [{} for _ in boards]

    # AS5600 feedback bindings: (csv_channel, chain_index, EncoderMap). Each
    # entry overwrites one driven joint's dead-reckoned angle with its
    # measured value when a fresh, finite reading is available.
    enc_bindings: list[tuple[int, int, EncoderMap]] = []
    if sensor is not None and cfg.sensors is not None:
        for enc in cfg.sensors.encoders:
            idx = ik.joint_index.get(enc.joint)
            if idx is not None:
                enc_bindings.append((enc.channel, idx, enc))

    # Zero-at-home: recapture each encoder's offset_deg from the reading
    # taken at launch so it maps to the home angle of its joint. This makes
    # the encoder feedback agree with the assumed home pose -- wherever the
    # arm physically sits now is declared home -- so the first feedback tick
    # leaves q at home instead of snapping it to an uncalibrated reading.
    if (enc_bindings and sensor is not None and cfg.sensors is not None
            and cfg.sensors.zero_at_home):
        deadline = time.monotonic() + 2.0
        raw_deg: list[float] = []
        while time.monotonic() < deadline and not stop_flag.is_set():
            raw_deg, stamp = sensor.snapshot()
            if (time.monotonic() - stamp) < SENSOR_STALE_SEC and raw_deg:
                break
            time.sleep(0.05)
        for ch, idx, enc in enc_bindings:
            r = raw_deg[ch] if 0 <= ch < len(raw_deg) else math.nan
            if math.isfinite(r):
                enc.offset_deg = offset_for_home(r, q[idx], enc)
                print(f"ik: zero-at-home {enc.joint}: raw={r:.2f}deg -> "
                      f"offset_deg={enc.offset_deg:.2f} "
                      f"(home={math.degrees(q[idx]):.2f}deg)", file=sys.stderr)
            else:
                print(f"ik: zero-at-home {enc.joint}: no reading on channel "
                      f"{ch}, keeping offset_deg={enc.offset_deg:.2f}",
                      file=sys.stderr)

    next_tick = time.monotonic()
    while not stop_flag.is_set():
        next_tick += period
        now = time.monotonic()
        sleep_for = next_tick - now
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic() + period

        if not js.connected:
            axes_snap: list[int] = []
        else:
            axes_snap = js.axes_snapshot()

        # --- Encoder feedback: replace dead-reckoned joint state with the
        # measured AS5600 angles so the Jacobian is evaluated at the real
        # pose. Stale frames (sensor unplugged) or NaN channels keep the
        # dead-reckoned value for that joint. ---
        if enc_bindings and sensor is not None:
            raw_deg, stamp = sensor.snapshot()
            if (time.monotonic() - stamp) < SENSOR_STALE_SEC:
                for ch, idx, enc in enc_bindings:
                    r = raw_deg[ch] if 0 <= ch < len(raw_deg) else math.nan
                    if math.isfinite(r):
                        q[idx] = encoder_rad(r, enc)

        # --- Commanded EE velocity from sticks ------------------------------
        sx = _stick(axes_snap, cfg.cartesian.axis_x, cfg.cartesian.deadzone,
                    cfg.cartesian.invert_x)
        sy = _stick(axes_snap, cfg.cartesian.axis_y, cfg.cartesian.deadzone,
                    cfg.cartesian.invert_y)
        sz = _stick(axes_snap, cfg.cartesian.axis_z, cfg.cartesian.deadzone,
                    cfg.cartesian.invert_z)
        v = np.array([sx * vmax[0], sy * vmax[1], sz * vmax[2]])

        # --- Resolved-rate: q_dot = J^T (J J^T + lam^2 I)^-1 v --------------
        if not np.any(v):
            q_dot = np.zeros(len(active))
        else:
            try:
                jac = ik.position_jacobian(q, active)
                a = jac @ jac.T + lam * lam * np.eye(3)
                q_dot = jac.T @ np.linalg.solve(a, v)
            except Exception as e:
                print(f"ik: jacobian solve failed: {e}", file=sys.stderr)
                q_dot = np.zeros(len(active))

        # Integrate dead-reckoned q forward, clamping at joint limits. When
        # a joint is pinned at a bound, the effective rate we send is the
        # clamped delta / dt so the firmware and our bookkeeping agree.
        eff_rate = np.zeros(len(active))
        for k, idx in enumerate(active):
            q_new = q[idx] + q_dot[k] * dt
            lo, hi = bounds[k]
            if math.isfinite(lo) and q_new < lo:
                q_new = lo
            elif math.isfinite(hi) and q_new > hi:
                q_new = hi
            eff_rate[k] = (q_new - q[idx]) / dt
            q[idx] = q_new

        # --- Per-board jog dispatch for IK joints --------------------------
        for bi, board in enumerate(boards):
            for jm in board.cfg.joints:
                if jm.target not in AXIS_TOKENS:
                    continue
                rate = eff_rate[col_of[jm.joint]]
                hz_float = rate * jm.steps_per_rad * (-1.0 if jm.invert else 1.0)
                vel_hz = _bucket_hz(hz_float)
                prev = last_hz[bi][jm.target]
                # Idle -> idle: nothing to send. The firmware's deadman
                # watchdog will halt motion if a nonzero jog stops being
                # refreshed, so we resend nonzero values every tick.
                if vel_hz == 0 and prev == 0:
                    continue
                last_hz[bi][jm.target] = vel_hz
                send(board, f"jog {jm.target} {vel_hz}\r\n", cfg.log)

            # --- Passthrough axes (non-IK DOFs) -----------------------------
            for ax in board.cfg.axes:
                if ax.target not in AXIS_TOKENS:
                    continue
                raw = (axes_snap[ax.index] / JS_MAX) if ax.index < len(axes_snap) else 0.0
                if ax.invert:
                    raw = -raw
                mag = abs(raw)
                dz = max(0.0, min(0.95, ax.deadzone))
                if mag < dz:
                    signed_v = 0.0
                else:
                    sign = 1.0 if raw > 0 else -1.0
                    norm = (mag - dz) / (1.0 - dz) * sign
                    signed_v = norm * ax.sensitivity
                new_hz = _bucket_hz(signed_v)
                prev = last_passthrough[bi].get(ax.target, (0, 0))[1]
                new_state = (1 if new_hz > 0 else (-1 if new_hz < 0 else 0),
                             abs(new_hz))
                if new_hz == 0 and prev == 0:
                    continue
                last_passthrough[bi][ax.target] = new_state
                send(board, f"jog {ax.target} {new_hz}\r\n", cfg.log)


# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------


def build_button_handler(cfg: Config, boards: list[Board]):
    """Fan toggles only -- the Rust host's button move/servo dispatch
    isn't ported here yet (IK mode doesn't need it). Add per-board
    bookkeeping if/when you wire up more."""
    fan_state: dict[tuple[int, int], bool] = {}

    def on_button(btn: int, pressed: bool):
        for bi, board in enumerate(boards):
            for f in board.cfg.fans:
                if f.index != btn:
                    continue
                key = (bi, f.target)
                if f.mode == "momentary":
                    fan_state[key] = pressed
                    cmd = f"fan {f.target} {'on' if pressed else 'off'}\r\n"
                    send(board, cmd, cfg.log)
                else:  # toggle
                    if pressed:
                        new = not fan_state.get(key, False)
                        fan_state[key] = new
                        cmd = f"fan {f.target} {'on' if new else 'off'}\r\n"
                        send(board, cmd, cfg.log)

    return on_button


def push_axis_config(cfg: Config, boards: list[Board]):
    """Mirror the Rust host's startup sequence: TMC UART config first,
    then accel, then enable. Driven joints and passthrough axes go
    through the same `tmc`/`accel` commands."""
    for board in boards:
        for jm in board.cfg.joints:
            if jm.target not in AXIS_TOKENS:
                continue
            if jm.microsteps != 0:
                send(board,
                     f"tmc {jm.target} {jm.microsteps} {jm.run_current_ma} "
                     f"{jm.hold_current_ma} {jm.hold_delay} "
                     f"{1 if jm.spreadcycle else 0} "
                     f"{1 if jm.interpolate else 0}\r\n",
                     cfg.log)
        for ax in board.cfg.axes:
            if ax.target not in AXIS_TOKENS:
                continue
            if ax.microsteps != 0:
                send(board,
                     f"tmc {ax.target} {ax.microsteps} {ax.run_current_ma} "
                     f"{ax.hold_current_ma} {ax.hold_delay} "
                     f"{1 if ax.spreadcycle else 0} "
                     f"{1 if ax.interpolate else 0}\r\n",
                     cfg.log)

    time.sleep(0.2)

    for board in boards:
        for jm in board.cfg.joints:
            if jm.target not in AXIS_TOKENS:
                continue
            send(board, f"accel {jm.target} {jm.accel}\r\n", cfg.log)
        for ax in board.cfg.axes:
            if ax.target not in AXIS_TOKENS:
                continue
            send(board, f"accel {ax.target} {ax.accel}\r\n", cfg.log)


def safe_stop(cfg: Config, boards: list[Board]):
    for board in boards:
        for a in ("x", "y", "z", "e"):
            send(board, f"jog {a} 0\r\n", cfg.log)
        for f in board.cfg.fans:
            send(board, f"fan {f.target} off\r\n", cfg.log)
        send(board, "disable all\r\n", cfg.log)
    time.sleep(0.15)


def main() -> int:
    here = Path(__file__).resolve().parent
    default_cfg = here.parent / "config-ik.toml"

    ap = argparse.ArgumentParser()
    ap.add_argument("config", nargs="?", default=str(default_cfg),
                    help=f"path to TOML config (default: {default_cfg})")
    args = ap.parse_args()

    cfg_path = Path(args.config).resolve()
    cfg = load_config(cfg_path)

    # URDF path is resolved relative to the config file's directory.
    urdf_path = (cfg_path.parent / cfg.urdf).resolve()
    if not urdf_path.is_file():
        print(f"urdf not found: {urdf_path}", file=sys.stderr)
        return 1

    print(f"loading urdf {urdf_path}", file=sys.stderr)
    ik = IkSolver(urdf_path, cfg.chain_elements, cfg.chain_base_type,
                  cfg.active_joints)
    print(f"ik chain: {len(ik.chain.links)} links, driving "
          f"{list(ik.joint_index.keys())}", file=sys.stderr)

    # Cross-check the config: every IK-driven joint mapping must point
    # at a URDF joint we actually discovered.
    cfg_joint_names = {jm.joint for b in cfg.boards for jm in b.joints}
    unknown = cfg_joint_names - set(ik.joint_index.keys())
    if unknown:
        print(f"config references joints not in IK chain "
              f"(check active_joints): {sorted(unknown)}", file=sys.stderr)
        return 1

    # home_joints may pin any chain joint (driven or held, e.g. the wrist),
    # so validate against all chain link names, not just the driven set.
    if cfg.cartesian.home_joints is not None:
        chain_names = {link.name for link in ik.chain.links}
        unknown_home = set(cfg.cartesian.home_joints) - chain_names
        if unknown_home:
            print(f"[cartesian].home_joints references joints not in IK chain: "
                  f"{sorted(unknown_home)}", file=sys.stderr)
            return 1

    boards: list[Board] = []
    for bcfg in cfg.boards:
        label = bcfg.name or bcfg.serial
        port = open_port(bcfg.serial)
        time.sleep(0.3)
        fw = handshake_version(port)
        print(f"board {label}: firmware v{fw}, host v{HOST_VERSION}",
              file=sys.stderr)
        if fw != HOST_VERSION:
            print(f"board {label}: version mismatch: firmware v{fw} != "
                  f"host v{HOST_VERSION}", file=sys.stderr)
            return 1
        board = Board(label=label, port=port, cfg=bcfg)
        spawn_drainer(board)
        boards.append(board)

    push_axis_config(cfg, boards)

    if cfg.enable_on_start:
        for board in boards:
            send(board, "enable all\r\n", cfg.log)

    # Optional AS5600 encoder feedback (firmware-sensors board on its own
    # serial port). Validate the encoder->joint mapping against the IK chain
    # before opening the port.
    sensor: SensorReader | None = None
    if cfg.sensors is not None and cfg.sensors.enabled:
        unknown_enc = ({e.joint for e in cfg.sensors.encoders}
                       - set(ik.joint_index.keys()))
        if unknown_enc:
            print(f"config [sensors] references joints not in IK chain "
                  f"(check active_joints): {sorted(unknown_enc)}",
                  file=sys.stderr)
            return 1
        sensor = SensorReader(cfg.sensors.serial, n_channels=4)
        sensor.start()
        print(f"sensors: {len(cfg.sensors.encoders)} AS5600 encoder(s) from "
              f"{cfg.sensors.serial}", file=sys.stderr)

    stop_flag = threading.Event()

    def on_sigint(_sig, _frm):
        stop_flag.set()

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    js = JoystickReader(cfg.device, build_button_handler(cfg, boards))
    js.start()

    integ = threading.Thread(
        target=integrate_loop,
        args=(cfg, ik, boards, js, stop_flag, sensor),
        name="integrator",
        daemon=True,
    )
    integ.start()

    total_joints = sum(len(b.joints) for b in cfg.boards)
    total_passthrough = sum(len(b.axes) for b in cfg.boards)
    n_encoders = len(cfg.sensors.encoders) if (sensor is not None and
                                               cfg.sensors is not None) else 0
    print(
        f"ik-host: device={cfg.device} poll_hz={cfg.poll_hz} "
        f"boards={len(cfg.boards)} ik_joints={total_joints} "
        f"passthrough_axes={total_passthrough} encoders={n_encoders}",
        file=sys.stderr,
    )
    print("Ctrl-C to stop.", file=sys.stderr)

    try:
        while not stop_flag.is_set():
            time.sleep(0.1)
    finally:
        if cfg.safe_stop_on_exit:
            safe_stop(cfg, boards)
        js.stop()
        if sensor is not None:
            sensor.stop()

    return 130


if __name__ == "__main__":
    sys.exit(main())
