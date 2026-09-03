# POW — Wheeled Bipedal Robot Simulator

A browser 3D simulator and log visualiser for the **POW** wheeled bipedal
robot, built on the robot's own control model rather than on a generic physics
engine. Open `index.html` — there is no build step and no network dependency.

![modes](../docs/simulator.png)

## What it does

**Simulate.** The firmware's model runs live in the browser: the five-bar leg
inverse kinematics sets the hip servos for the commanded height, the seven
upper links are aggregated into an equivalent body (CoM offset and inertia
tensor), and the gain-scheduled LQR closes the loop around the reduced 4-state
dynamics — all at the firmware's 8 ms sampling period. You fly it with the
keyboard; it balances, or falls over and resets if you shove it hard enough.

**Replay log.** The same robot is driven from flight logs recorded on the real
hardware, so you can watch what actually happened during a balancing run, a
squat, a top-speed pass or a yaw-rate test, with the telemetry beside it.

## Controls

| Key       | Action                                    |
| --------- | ----------------------------------------- |
| `W` / `S` | forward / reverse velocity command (±1 m/s) |
| `A` / `D` | yaw rate command (±1 rad/s)               |
| `Q` / `E` | raise / lower the body (70–200 mm)        |
| `Space`   | hold to crouch to minimum height          |
| `P`       | shove the robot, to watch the LQR reject it |
| `R`       | reset                                     |

Arrow keys mirror `Q`/`E`/`A`/`D`, and the on-screen pads work with mouse and
touch. Uncheck *Chase camera* to orbit the robot freely.

## Where the numbers come from

Everything physical is transcribed from
[SeungbinOh/Pow_WBR_Project](https://github.com/SeungbinOh/Pow_WBR_Project):

| This repo                | Ported from                             |
| ------------------------ | --------------------------------------- |
| `src/params.js`          | `ESP32/WBR_Control/Params.h` — link lengths, masses, CoM offsets, inertia tensors, operating limits |
| `src/params.js` (gains)  | `ESP32/WBR_Control/VYBController.h` — the 14-entry LQR gain schedule |
| `src/pol.js`             | `ESP32/WBR_Control/POL.h` — inverse and forward kinematics, CoM/inertia aggregation, mass matrix and nonlinear effects |
| `src/lqr.js`             | `ESP32/WBR_Control/VYBController.h` — gain interpolation, torque saturation |
| `index.js` control order | `ESP32/WBR_Control/WBR_Control.ino` — the sequence of the main loop |
| `data/*.csv`             | `MATLAB/log_plot/` and `MATLAB/test_data/`, thinned to ~4000 samples and reduced to the state/reference/torque columns |

The chassis box you see is not a guess: its dimensions are recovered from the
body inertia tensor in `Params.h` (207 × 144 × 114 mm), and it is drawn at the
body's real CoM offset.

### Model summary

State `x = [θ, θ̇, v, ψ̇]` — body pitch, pitch rate, forward velocity of the
wheel axle, yaw rate. Input `u = [τ_RW, τ_LW]`, the two wheel torques.
Dynamics `M(θ) q̈ + nle = B u`, with

```
B = [  1    -1  ]
    [ -1/R   1/R]
    [ -L/R  -L/R]
```

`M` and `nle` depend on the commanded height through the aggregated body: at
each control step the leg pose is re-solved, the CoM offset `p_bcom` and
inertia `I_B_B` are recomputed, and the LQR gain is re-interpolated. The pitch
reference is the equilibrium angle `atan(-p_bcom_x / (h + p_bcom_z))`, which
puts the centre of mass over the wheel contact — the same thing
`WBR_Control.ino` does every loop.

### Three deliberate deviations from the firmware

All three are bugs in the original C++ that a simulator cannot live with. They
are marked in the source where they occur.

1. **`POL::calculate_com_and_inertia` accumulates.** `p_bcom` and `I_B_B` are
   never zeroed before the summation, so on the hardware they grow without
   bound across calls. Here they are reset each time.
2. **`POL::solve_inverse_kinematics` overwrites `phi` with radians.** The member
   is documented as degrees and is converted in place on every call, so the
   commanded roll shrinks by a factor of 57.3 per control step. Here the
   commanded value is kept in degrees and converted locally.
3. **`POL::calculate_nle` closes a parenthesis early in the yaw equation.** The
   last three terms end up added straight into `nle(2)` instead of being scaled
   by `ψ̇θ̇`. The result is a constant ~2.3 rad/s² yaw acceleration at rest with
   no torque applied. The dangling terms are exactly twice the matching ones in
   the pitch equation, which identifies it as a precedence slip; they are
   grouped as intended. Without this fix the robot spins on the spot.

Note also that the inverse and forward kinematics are not perfectly consistent
with each other in the original: the IK solves a triangle that the FK chain
then reproduces to within about 1 mm on the calf length. That discrepancy is
inherited as-is, since it is what the hardware actually commands.

## Layout

```
index.html          page, styling, key pads, import map
index.js            scene setup, control loop, UI wiring
src/params.js       physical parameters and the LQR gain schedule
src/mat.js          3x3 matrix and vector helpers
src/pol.js          kinematics, body aggregation, dynamics
src/lqr.js          gain-scheduled LQR
src/robot3d.js      three.js robot assembly and scene
src/hud.js          rolling telemetry charts
src/logplayer.js    CSV flight-log parsing and playback
data/               flight logs from the real robot
vendor/             three.js r169 (see vendor/README.md)
```

## Licence and attribution

Three.js is MIT, vendored under `vendor/` with its licence. The robot model,
control gains and flight logs originate from the POW capstone project by
Seungbin Oh, Jungbin Park and Woodaengtang; see that repository for its own
terms. The viewer's shape — full-bleed canvas with on-screen WASD pads —
follows the NavBot-EN01 simulator.
