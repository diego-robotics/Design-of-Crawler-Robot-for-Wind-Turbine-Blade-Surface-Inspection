# Codes — Suction-Based Crawler for Wind Turbine Blade Inspection

Two subfolders, two complementary validation efforts:
- **`simulation/`** — validates whole-robot coordination (can six legs walk a tripod gait and attach/detach in sync).
- **`prototype trials/`** — validates the physical adhesion principle itself (real vacuum, real leakage, real surfaces) on a single-leg hardware prototype.

Neither alone proves the full concept: simulation's adhesion is a physically ideal rigid joint (no air leakage modeled); the prototype is one leg, not six. Together they cover the two halves of the argument.

---

## `simulation/`

Full detail in `simulation/README.md`. Two packages:

- **`gait + adhesion`** — `assembly_with_links` (URDF, world, launch), `gait_control.py`, and a custom `hexapod_link_attacher` Gazebo plugin (built because two existing plugins weren't usable — see the simulation README). Adhesion confirmed working in isolated testing; intermittently stalls under continuous gait operation. Included to show the attempt and the debugging done.
- **`gait only`** — same robot/gait, adhesion plugin fully removed. Fully reliable fallback demo.

Run either with:
```bash
colcon build
source install/setup.bash
ros2 launch assembly_with_links gazebo_launch.py
```

---

## `prototype trials/`

Single-leg hardware rig: Raspberry Pi 5 + Arduino (Elegoo UNO) + 3 servos + vacuum pump/solenoid valve + pressure sensor. Pi reads an Xbox controller and talks to the Arduino over serial; Arduino handles real-time servo control and sensor sampling.

- **`spider_leg_keyboard_control.ino`** — Arduino firmware. Single-character serial protocol (`q/e`, `a/d`, `z/c` toggle joints 1–3; `w/s/x` home them; `v/V` vacuum on, `b/B` vacuum off with a 0.5s valve-vent delay; `?` status). Sensor voltage streamed every 200ms.
- **`main.py`** — Raspberry Pi controller. Maps Xbox sticks/buttons to the Arduino protocol. `A` button toggles vacuum **and** starts/stops experiment recording: while off, keeps a rolling baseline voltage; while on, logs voltage vs. time. On stop, computes average drop, % drop, std dev, and stabilization time, applies a pass/fail threshold (avg < 3.9V), prompts for trial conditions (surface, weight, angle, curvature, wet/dry, induced failure), and appends a row to `payload_experiments.xlsx`.
- **`Instructions_for_using_the_prototype.docx`** — bench setup/operating checklist (power up sequence, cabling, first-use functional check). Pi login: `diego`/`diego`.
- **Spreadsheets** — `payload_experiments.xlsx` (auto-generated trial log, as above). A second spreadsheet exists in the actual folder but wasn't uploaded, so its contents aren't described here — document it directly if it holds a distinct dataset.
