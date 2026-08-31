# Hexapod Suction Crawler — Simulation README

## Objective

Offshore wind turbine blades need regular inspection for structural damage (cracks, delamination, erosion), currently done via rope-access technicians (slow, expensive, risky) or drones (safe, but no close-contact sensing). This project simulates a six-legged suction-adhesion crawler robot that could walk directly across a blade surface using a tripod gait, carrying inspection sensors to points existing methods struggle to reach. Simulation validates the mechanical/control concept before any physical build.

## Toolchain

- ROS 2 Humble, `gait_control.py` node (same middleware a real onboard computer would run)
- Gazebo Classic 11.10.2, `ros2_control` + `joint_trajectory_controller`
- Robot model exported from SolidWorks CAD to URDF

## Robot Model

Hexapod, legs numbered 1–6, tripod gait groups A={1,3,5}/B={2,4,6}. Each leg: `l1→j1→l2→j2→l3→j3→l4→j4→l5→j5→l6`(cup). `j1`-`j3` actively driven; `j4`/`j5` were passive, then briefly actuated, now fixed rigid joints (see below). Early CAD-export errors in legs 3/6's joint origins were fixed via mesh registration.

## Gait Controller

Time-based tripod gait — deliberately simple and dependency-free (no sensor/confirmation gating) for robustness. A more elaborate confirmation-gated state machine was built for adhesion integration, then reverted in favor of this simpler design once reliability mattered more than adhesion-gait coordination.

## Suction Adhesion: Three Attempts

1. **IFRA LinkAttacher** (existing plugin) — only supports one attachment in the whole simulated world at a time, confirmed by exhaustive testing. Fatal for a tripod gait, which needs three simultaneous attachments. Abandoned.
2. **`gazebo_ros_vacuum_gripper`** (existing plugin, per-leg instances) — `switch()` calls failed 100% of the time despite confirmed physical contact. Root cause never found despite extensive testing (friction, contact sensors, reboots all ruled out). Abandoned.
3. **Custom `hexapod_link_attacher` plugin** — combines LinkAttacher's rigid-joint mechanism with per-leg independent instances. Four bugs found and fixed, two of them genuine C++ concurrency bugs diagnosed via live GDB backtraces and Gazebo's own crash output rather than guesswork:
   - `CreateJoint()`'s return value was unreliable → verify via `GetJoint()` instead.
   - `Detach()` alone didn't release the physics constraint → added `RemoveJoint()`.
   - A lock-ordering **deadlock** (a custom mutex conflicting with Gazebo's own internal locking) → removed the unnecessary mutex.
   - A **data race** (concurrent attach/detach calls on the same leg, since the ROS server is multi-threaded) → added a narrowly-scoped mutex protecting only the plugin's own state, never held while calling into Gazebo.

**Note:** research confirmed Gazebo Classic is EOL (Jan 2025) and modern `gz-sim` has a built-in `DetachableJoint` system suited to this exact problem. Continued with Classic given sunk cost and near-working state; worth reconsidering later.

## Deadline Simplification

Ahead of a presentation, suction adhesion was stripped out entirely to guarantee a reliable gait-only demo. `j4`/`j5` made fully fixed (eliminated uncontrolled cup spinning). Found and fixed a leftover 0.8kg cup mass (artifact of an abandoned self-leveling experiment) causing standing-pose sag. Established a useful diagnostic: `/joint_states` effort pinned at limits + sign-flipping every tick = gain instability, not torque shortage (don't just add torque).

## Demo Environment

Procedurally generated tapered/twisted airfoil blade mesh (Python/trimesh), offshore sky+sea backdrop, spawn position calculated from blade geometry. Black/yellow robot color scheme required Gazebo-specific `<gazebo reference>` material blocks — URDF's own `<material><color>` tags are not reliably read by Gazebo's renderer (works in RViz only).

## Re-integrating Adhesion

Reintroduced the custom plugin on the stable base. Found that `j4`/`j5` being fixed joints causes URDF→SDF conversion to lump `l6` into `l4` (standard fixed-joint-chain behavior) — link `{leg}-l6` no longer exists in Gazebo; attach calls now target `{leg}-l4` instead (physically equivalent, same rigid body). Also fixed: a `MultiThreadedExecutor` alone doesn't parallelize a ROS 2 node — callbacks need explicit separate callback groups, or they still run one-at-a-time.

## Current Status

- **Gait alone:** reliable.
- **Adhesion mechanism:** confirmed working in isolated CLI testing, including simultaneous multi-leg attachment.
- **Adhesion under continuous gait:** intermittently stalls (all legs end up stuck attaching) — not yet conclusively diagnosed; needs a fresh GDB backtrace or synced video capture at the moment of failure to resolve, same approach that solved every prior concurrency bug here.

## Future Work

- Diagnose the remaining intermittent stall.
- Reintroduce foot leveling (`j4`/`j5`) for genuine surface-curvature adaptation.
- Test on more realistic blade geometry.
- Reconsider `gz-sim` migration if Classic-specific issues keep dominating.
- Tune gait speed further once adhesion is fully reliable.
