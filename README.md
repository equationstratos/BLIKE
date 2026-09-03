# BLIKE

## Simulator

[`Simulator/`](Simulator/) — a browser 3D simulator and flight-log visualiser
for the POW wheeled bipedal robot. It runs the robot's own control stack: the
five-bar leg kinematics, the equivalent-body CoM and inertia aggregation, and
the gain-scheduled LQR balance controller, all ported from the
[Pow_WBR_Project](https://github.com/SeungbinOh/Pow_WBR_Project) firmware and
stepped at its 8 ms period.

Open `Simulator/index.html` in a browser — no build, no network access needed.
Drive it with `WASD`, change ride height with `Q`/`E`, crouch with `Space`,
shove it with `P`. Switch to *Replay log* to play back runs recorded on the
real hardware.

See [`Simulator/README.md`](Simulator/README.md) for the model, the mapping
back to the firmware files, and the three firmware bugs the simulator corrects.
