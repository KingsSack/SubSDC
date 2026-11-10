# BlueROV1

A Gazebo model of the original Blue Robotics BlueROV (2015 kit, "BlueROV1"), derived
from the `bluerov2` model. It is meant to be flown by ArduSub with `FRAME_CONFIG = 0`
(BLUEROV1); `run_tests.py --layout bluerov1` sets that automatically.

## Sources

- [BlueROV documentation](https://docs.bluerobotics.com/bluerov/): kit contents, dimensions,
  thruster numbering and prop assignment.
- Blue Robotics' BlueROV R1 CAD (`BROV-R1.STL`, linked from the docs page): thruster positions.

> The docs page's specification table says it is "given for the BlueROV with T100 Thrusters"
> (2.36 kgf per thruster). That table comes from the older T100 version. The kit contents
> list and the assembly notes say the kit ships with **6 x T200**, so this model uses T200s.

## What changed vs. `bluerov2`

| Property | bluerov2 | bluerov1 | Source |
|---|---|---|---|
| Envelope (L x W x H) | 0.457 x 0.338 x 0.254 m | 0.483 x 0.330 x 0.267 m | BlueROV spec table (19 x 13 x 10.5 in); CAD agrees (0.483 x 0.337 x 0.266) |
| Mass | 10 kg | **6.5 kg** | **Estimate, not verified.** The spec gives 3.73 kg "no electronics" but does not say whether that includes thrusters/enclosure. Add 6 x T200 (~0.34 kg each), electronics and battery as appropriate. |
| Inertia (ixx, iyy, izz) | 0.099, 0.178, 0.269 | 0.065, 0.129, 0.185 | Scaled from bluerov2 by mass and envelope |
| CoM | (0, 0, 0.011) | (-0.007, 0, 0.011) | x: zero-pitch point for ArduSub's vertical-thruster factors (below); z: estimate |
| Buoyancy box | 0.457 x 0.338 x 0.065 | 0.483 x 0.330 x 0.041 at x = -0.007 | Sized for ~0.5% positive buoyancy, like bluerov2; centred over the CoM |
| Thrusters | 6 x T200, +-50 N | 6 x T200, +-50 N | Kit contents list; same thrust scaling as bluerov2 |
| Thruster layout | 4 vectored horizontal + 2 vertical | 2 forward + 3 vertical + 1 lateral | ArduSub `SUB_FRAME_BLUEROV1` mixer, BlueROV manual |
| Quadratic damping | x -33.7, y -54.2, z -73.2 | x -34.7, y -60.1, z -75.5 | Scaled by projected area (estimate) |

## Thruster layout (Gazebo body frame: x forward, y left, z up)

Positions are measured from the BROV-R1 CAD (thruster nozzle centres) and given relative to
the frame's bounding-box centre, which is the `base_link` origin.

| # | Position (m) | Pushes | Prop | ArduSub factors (roll, pitch, yaw, thr, fwd, lat) |
|---|---|---|---|---|
| 1 | (-0.006, -0.111, +0.039) | forward | reversed | (0, 0, -1, 0, 1, 0) |
| 2 | (-0.006, +0.111, +0.039) | forward | standard | (0, 0, +1, 0, 1, 0) |
| 3 | (+0.178, -0.111, +0.016) | up | reversed | (-0.5, 0.5, 0, 0.45, 0, 0) |
| 4 | (+0.178, +0.111, +0.016) | up | standard | (0.5, 0.5, 0, 0.45, 0, 0) |
| 5 | (-0.174, 0.000, -0.072) | up | standard | (0, -1, 0, 1, 0, 0) |
| 6 | (+0.039, 0.000, -0.070) | right | standard | (-0.25, 0, 0, 0, 0, 1) |

The thrust directions were checked against the ArduSub mixer signs. The prop assignment
follows the manual: only the right horizontal (1) and right vertical (3) thrusters get the
counter-rotating prop. In this model the props are visual only, because with
`velocity_control` the Thruster system applies no reaction torque.

The CoM x position (-0.007) is the point where ArduSub's throttle factors produce no pitch
moment: 2 x 0.45 x (0.178 + 0.007) = 1.0 x (0.174 - 0.007). It is also the longitudinal
station of the horizontal thrusters, which suggests this is the intended balance point.

## Known limitations

- No BlueROV1 mesh; the body is shown as a translucent box (the T200 prop meshes are reused).
- Mass, vertical CoM, inertia and damping are estimates, not measurements.
  Tune them (or use `mass_kg` / `com_offset_m` in `run_tests.py`) if you have real data.
- T200 reverse thrust is weaker than forward (~4.1 vs ~5.25 kgf at 16 V); like bluerov2,
  the ArduPilot plugin models it as symmetric at +-50 N. At 12 V a T200 gives ~3.7 kgf
  (~36 N) forward, so lower the `multiplier` if you simulate a 3S battery.
