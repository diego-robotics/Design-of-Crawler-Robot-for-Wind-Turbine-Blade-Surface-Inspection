#!/usr/bin/env python3
"""
Tripod-gait keyboard teleop for the 6-legged assembly_with_links crawler.

STRIPPED VERSION -- suction adhesion physics (hexapod_link_attacher,
contact sensors, active leveling) removed ahead of a presentation
deadline, to validate the tripod gait alone first, with no external
plugin dependency that could fail/crash/deadlock the sim. j4/j5 are back
to fully passive (gravity/damping only). Once gait is confirmed solid,
suction adhesion can be re-added -- see chat history / prior versions of
crawler_hexapod.world, assembly_with_links.urdf, and this file for the
hexapod_link_attacher-based implementation.

Hold a direction key to walk that way (forward / backward / left / right /
the four diagonals). Release it and the robot returns to a neutral standing
pose. CTRL-C to quit.

    q  w  e      forward-left   forward    forward-right
    a     d      left                      right
    z  s  c      backward-left  backward   backward-right
"""

import math
import sys
import select
import threading
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None


# ---------------------------------------------------------------------------
# Robot geometry
# ---------------------------------------------------------------------------
LEG_IDS = [1, 2, 3, 4, 5, 6]
TRIPOD_A = {1, 3, 5}
TRIPOD_B = {2, 4, 6}

# Matches the -entity name used at spawn (gazebo_launch.py) and the
# link-naming convention used throughout the URDF. Cups attach to the
# blade model (see crawler_hexapod.world), not ground_plane.
ROBOT_MODEL_NAME = 'assembly_with_links'
SURFACE_MODEL_NAME = 'blade'
SURFACE_LINK_NAME = 'link'


def cup_link_name(leg):
    return f'{leg}-l6'


# Mounting yaw of each leg around the body (rad), taken directly from the
# <origin rpy="... ... Z"/> of each 'X-j0' fixed joint in the URDF.
LEG_YAW = {
    1: 0.25431,
    2: 1.30150,
    3: 2.34870,
    4: -2.88730,
    5: -1.84010,
    6: -0.79289,
}

ACTIVE_JOINT_SUFFIXES = ['j1', 'j2', 'j3']
JOINT_NAMES = []
for _leg in LEG_IDS:
    JOINT_NAMES += [f'{_leg}-{s}' for s in ACTIVE_JOINT_SUFFIXES]


# ---------------------------------------------------------------------------
# Stance / gait tuning -- adjust these to taste once you see it walk
# ---------------------------------------------------------------------------
STAND_J2 = 0.8
STAND_J3 = 0.8

SWEEP_AMPLITUDE = 1.3     # rad, max +/- coxa (j1) swing at full speed -- j1 limit is +/-1.57, leaves margin
LIFT_AMOUNT = 1.3         # rad, how much j2/j3 change during swing to lift the foot -- j3's upper limit was widened to 2.2 in the URDF specifically to accommodate this (STAND_J3=0.8 + 1.3 = 2.1, just under the new limit)

# NOTE -- verify both of these signs the first time you run this in sim:
#   SWEEP_SIGN: if "forward" actually walks the robot backward, flip this to -1.0.
#   LIFT_SIGN:  if the swinging foot digs into the ground instead of lifting
#               clear of it, flip this to -1.0.
SWEEP_SIGN = 1.0
LIFT_SIGN = 1.0

CYCLE_PERIOD = 2.5         # seconds for one full stance+swing cycle
CONTROL_PERIOD = 0.05      # seconds between trajectory updates (20 Hz)
STEP_TIME = CONTROL_PERIOD * 2.0  # time_from_start given to each streamed point
TABLE_REFRESH_PERIOD = 0.3        # seconds between status-table redraws

# While standing still with an unchanged target, don't keep re-arming an
# urgent STEP_TIME deadline every control tick -- see _control_loop for
# why. STANDING_HOLD_TIME is generous/non-urgent since nothing is actually
# supposed to move; STANDING_KEEPALIVE_PERIOD is just a safety resend in
# case a message was ever dropped, not a real control-rate requirement.
STANDING_HOLD_TIME = 1.0
STANDING_KEEPALIVE_PERIOD = 2.0

KEY_TIMEOUT = 0.4          # seconds with no key repeat before treating a
                           # direction as "released" and stopping


# Keyboard -> (dx, dy) in body frame. dx = forward(+)/backward(-),
# dy = left(+)/right(-). Diagonals fall out automatically from combining them.
KEY_DIRECTIONS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'q': (1.0, 1.0),
    'e': (1.0, -1.0),
    'z': (-1.0, 1.0),
    'c': (-1.0, -1.0),
}

DIRECTION_LABELS = {
    (0.0, 0.0): 'standing',
    (1.0, 0.0): 'forward',
    (-1.0, 0.0): 'backward',
    (0.0, 1.0): 'left',
    (0.0, -1.0): 'right',
    (1.0, 1.0): 'forward-left',
    (1.0, -1.0): 'forward-right',
    (-1.0, 1.0): 'backward-left',
    (-1.0, -1.0): 'backward-right',
}

CONTROLS_TEXT = """  q  w  e      forward-left   forward    forward-right
  a     d      left                      right
  z  s  c      backward-left  backward   backward-right

  Hold a key to walk that way. Release it to stand still.
  CTRL-C to quit."""


# ---------------------------------------------------------------------------
# Gait math
# ---------------------------------------------------------------------------
class GaitEngine:
    """Computes joint positions for all 18 active joints (j1-j3 x 6 legs)
    given a desired body-frame walking direction and the current time.
    Pure time-based tripod gait -- no suction/adhesion dependency of any
    kind, deliberately, for this stripped-down version. Reports each leg's
    current mode ('stand' / 'stance' / 'swing') for diagnostic display
    only; nothing downstream depends on it.
    """

    def __init__(self):
        self.start_time = time.monotonic()

    def compute(self, dx, dy):
        """Return (positions, leg_mode):
          positions: {joint_name: position} for all 18 active joints
          leg_mode:  {leg_id: 'stand' | 'stance' | 'swing'}
        """
        positions = {}
        leg_mode = {}
        magnitude = math.hypot(dx, dy)

        if magnitude < 1e-3:
            # No direction commanded: hold the neutral standing pose.
            for leg in LEG_IDS:
                positions[f'{leg}-j1'] = 0.0
                positions[f'{leg}-j2'] = STAND_J2
                positions[f'{leg}-j3'] = STAND_J3
                leg_mode[leg] = 'stand'
            return positions, leg_mode

        heading = math.atan2(dy, dx)  # desired direction in body frame
        speed = min(magnitude, 1.0)

        t = time.monotonic() - self.start_time
        cycle_phase = (t % CYCLE_PERIOD) / CYCLE_PERIOD  # 0..1

        for leg in LEG_IDS:
            in_group_a = leg in TRIPOD_A
            local_phase = cycle_phase if in_group_a else (cycle_phase + 0.5) % 1.0

            align = math.cos(heading - LEG_YAW[leg])
            amp = SWEEP_SIGN * SWEEP_AMPLITUDE * align * speed

            if local_phase < 0.5:
                # Stance: planted, sweeping from +amp to -amp, driving the body.
                s = local_phase / 0.5
                j1 = amp * (1.0 - 2.0 * s)
                j2 = STAND_J2
                j3 = STAND_J3
                leg_mode[leg] = 'stance'
            else:
                # Swing: lifted, sweeping back from -amp to +amp to reset.
                s = (local_phase - 0.5) / 0.5
                j1 = amp * (2.0 * s - 1.0)
                lift = LIFT_SIGN * LIFT_AMOUNT * math.sin(math.pi * s)
                j2 = STAND_J2 - lift
                j3 = STAND_J3 + lift
                leg_mode[leg] = 'swing'

            positions[f'{leg}-j1'] = j1
            positions[f'{leg}-j2'] = j2
            positions[f'{leg}-j3'] = j3

        return positions, leg_mode


# ---------------------------------------------------------------------------
# Suction cup attach/detach (per-leg hexapod_link_attacher plugin instances)
# ---------------------------------------------------------------------------
class SuctionCupManager:
    """Controls six independent hexapod_link_attacher plugin instances (one
    per leg, declared in crawler_hexapod.world), each namespaced /leg{N}
    with its own ATTACHLINK/DETACHLINK services (linkattacher_msgs).

    Deliberately simple: fire-and-forget async calls, driven directly by
    GaitEngine's leg_mode ('stance'/'stand' -> attach, 'swing' -> detach)
    on every control tick, with only a basic retry cooldown to avoid
    spamming the same request. No confirmation-gated state machine, no
    waiting for cup state before allowing leg motion -- the gait's own
    timing is untouched, exactly as it was with no adhesion mechanism at
    all. Good enough for basic reliable operation without the complexity
    of the earlier, more elaborate version.
    """

    UNKNOWN = 'UNKNOWN'
    ATTACHED = 'ATTACHED'
    DETACHED = 'DETACHED'
    ATTACHING = 'ATTACHING'
    DETACHING = 'DETACHING'

    RETRY_COOLDOWN = 1.0  # seconds between repeated attempts of the same state

    def __init__(self, node: Node):
        self.node = node
        self.state = {leg: self.UNKNOWN for leg in LEG_IDS}
        self._last_commanded = {leg: None for leg in LEG_IDS}
        self._last_attempt_time = {leg: 0.0 for leg in LEG_IDS}
        self._attach_clients = {}
        self._detach_clients = {}
        self._services_warned = set()

        for leg in LEG_IDS:
            self._attach_clients[leg] = node.create_client(AttachLink, f'/leg{leg}/ATTACHLINK')
            self._detach_clients[leg] = node.create_client(DetachLink, f'/leg{leg}/DETACHLINK')

    def attach(self, leg):
        self._set(leg, True)

    def detach(self, leg):
        self._set(leg, False)

    def _set(self, leg, on):
        target = self.ATTACHED if on else self.DETACHED
        if self.state[leg] == target:
            return

        if self._last_commanded[leg] == on:
            elapsed = time.monotonic() - self._last_attempt_time[leg]
            if elapsed < self.RETRY_COOLDOWN:
                return

        client = self._attach_clients[leg] if on else self._detach_clients[leg]
        if not client.service_is_ready():
            if leg not in self._services_warned:
                self.node.get_logger().warn(
                    f'Leg {leg}: /leg{leg}/ATTACHLINK or DETACHLINK not available yet -- '
                    'is hexapod_link_attacher loaded for this leg? Will keep retrying.'
                )
                self._services_warned.add(leg)
            return
        self._services_warned.discard(leg)

        self._last_commanded[leg] = on
        self._last_attempt_time[leg] = time.monotonic()

        if on:
            req = AttachLink.Request()
            req.model1_name = ROBOT_MODEL_NAME
            req.link1_name = cup_link_name(leg)
            req.model2_name = SURFACE_MODEL_NAME
            req.link2_name = SURFACE_LINK_NAME
            self.state[leg] = self.ATTACHING
            future = client.call_async(req)
        else:
            req = DetachLink.Request()
            req.model1_name = ROBOT_MODEL_NAME
            req.link1_name = cup_link_name(leg)
            req.model2_name = SURFACE_MODEL_NAME
            req.link2_name = SURFACE_LINK_NAME
            self.state[leg] = self.DETACHING
            future = client.call_async(req)

        future.add_done_callback(lambda f, leg=leg, on=on: self._on_done(leg, on, f))

    def _on_done(self, leg, on, future):
        settled_ok = self.ATTACHED if on else self.DETACHED
        settled_fail = self.DETACHED if on else self.ATTACHED
        try:
            result = future.result()
            self.state[leg] = settled_ok if result.success else settled_fail
        except Exception as exc:
            self.node.get_logger().error(
                f'Leg {leg}: {"attach" if on else "detach"} service call raised {exc}'
            )
            self.state[leg] = self.UNKNOWN


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class GaitControllerNode(Node):
    def __init__(self):
        super().__init__('gait_controller_node')
        self.publisher_ = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        self.engine = GaitEngine()
        self.cups = SuctionCupManager(self)

        self._lock = threading.Lock()
        self.dx = 0.0
        self.dy = 0.0
        self.last_key_time = time.monotonic()
        self.current_direction = (0.0, 0.0)
        self._last_published_positions = None
        self._last_publish_time = 0.0

        self.create_timer(CONTROL_PERIOD, self._control_loop)
        self.create_timer(TABLE_REFRESH_PERIOD, self._draw_status_table)
        self.get_logger().info('Gait controller ready (gait-only, no suction adhesion).')

    def set_direction(self, dx, dy):
        """Thread-safe: called from the keyboard-reading thread (and, later,
        from a /joy callback) to update the currently-commanded direction.
        """
        with self._lock:
            self.dx = dx
            self.dy = dy
            self.last_key_time = time.monotonic()

    def _control_loop(self):
        with self._lock:
            dx, dy = self.dx, self.dy
            timed_out = (time.monotonic() - self.last_key_time) > KEY_TIMEOUT

        if timed_out:
            dx, dy = 0.0, 0.0

        self.current_direction = (dx, dy)

        positions_by_name, leg_mode = self.engine.compute(dx, dy)
        self._leg_mode = leg_mode  # for status table only

        for leg in LEG_IDS:
            if leg_mode[leg] == 'swing':
                self.cups.detach(leg)
            else:  # 'stance' or 'stand'
                self.cups.attach(leg)

        standing_still = math.hypot(dx, dy) < 1e-3

        if standing_still:
            # While genuinely standing still, positions_by_name never
            # changes tick to tick -- don't keep re-arming an urgent
            # STEP_TIME deadline every control tick for an already-reached
            # target; that reacts aggressively to small position noise
            # instead of just holding still.
            now = time.monotonic()
            already_holding = (
                self._last_published_positions is not None
                and all(
                    abs(positions_by_name[name] - self._last_published_positions[name]) < 1e-6
                    for name in JOINT_NAMES
                )
            )
            if already_holding and (now - self._last_publish_time) < STANDING_KEEPALIVE_PERIOD:
                return
            hold_time = STANDING_HOLD_TIME
        else:
            hold_time = STEP_TIME

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [positions_by_name[name] for name in JOINT_NAMES]
        hold_sec = int(hold_time)
        hold_nsec = int(round((hold_time - hold_sec) * 1e9))
        point.time_from_start = Duration(sec=hold_sec, nanosec=hold_nsec)
        msg.points.append(point)
        self.publisher_.publish(msg)

        self._last_published_positions = dict(positions_by_name)
        self._last_publish_time = time.monotonic()

    def _draw_status_table(self):
        dx, dy = self.current_direction
        label = DIRECTION_LABELS.get((dx, dy), f'dx={dx:+.1f} dy={dy:+.1f}')
        leg_mode = getattr(self, '_leg_mode', {leg: 'stand' for leg in LEG_IDS})

        lines = []
        lines.append('=' * 64)
        lines.append(' Hexapod locomotion + suction cup status'.ljust(64))
        lines.append('=' * 64)
        for leg in (1, 2, 3):
            other = leg + 3
            s1 = f"{leg_mode.get(leg, '?'):<8}{self.cups.state[leg]:<10}"
            s2 = f"{leg_mode.get(other, '?'):<8}{self.cups.state[other]:<10}"
            lines.append(f'  Leg {leg}: {s1}  Leg {other}: {s2}')
        lines.append('-' * 64)
        lines.append(CONTROLS_TEXT)
        lines.append('-' * 64)
        lines.append(f'  Direction: {label}')
        lines.append('=' * 64)

        # Clear screen + move cursor home, then redraw. \r\n throughout so
        # this stays correct even if the terminal happens to be caught in
        # raw mode by the keyboard-reading thread at the same instant.
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write('\r\n'.join(lines) + '\r\n')
        sys.stdout.flush()


# ---------------------------------------------------------------------------
# Keyboard input (POSIX only -- Linux/macOS terminal raw mode)
# ---------------------------------------------------------------------------
def get_key(settings, timeout):
    """Blocks up to `timeout` seconds for a single keypress; returns '' on
    timeout with nothing pressed. Standard raw-terminal read pattern (the
    same one used by ROS's own teleop_twist_keyboard).
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def keyboard_loop(node):
    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings, KEY_TIMEOUT)
            if key == '\x03':  # CTRL-C
                break
            direction = KEY_DIRECTIONS.get(key.lower())
            if direction is not None:
                node.set_direction(*direction)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    if termios is None:
        print('This keyboard teleop requires a POSIX terminal (Linux/macOS).',
              file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=args)
    node = GaitControllerNode()

    key_thread = threading.Thread(target=keyboard_loop, args=(node,), daemon=True)
    key_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
