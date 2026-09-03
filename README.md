# BLIKE

## Simulator

[`Simulator/`](Simulator/) — a browser 3D simulator and flight-log visualiser
for the POW wheeled bipedal robot. It runs the robot's own control stack: the
five-bar leg kinematics, the equivalent-body CoM and inertia aggregation, the
extended Kalman filter fed by synthetic IMU and encoder readings, and the
gain-scheduled LQR balance controller closing the loop around the estimate —
all ported from the
[Pow_WBR_Project](https://github.com/SeungbinOh/Pow_WBR_Project) firmware and
stepped at its 8 ms period.

Open `Simulator/index.html` in a browser — no build, no network access needed.
Drive it with `WASD`, change ride height with `Q`/`E`, crouch with `Space`,
shove it with `P`. Switch to *Replay log* to play back runs recorded on the
real hardware.

See [`Simulator/README.md`](Simulator/README.md) for the model, the mapping
back to the firmware files, and the five firmware bugs the simulator corrects —
each one confirmed numerically rather than assumed.
