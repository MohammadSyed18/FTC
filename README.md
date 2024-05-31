# FTC control toolkit

Hardware-independent Java control and navigation components for local robotics experiments. This repository focuses on encoder localization and waypoint control; the FTC SDK and motor adapters are separate integration work.

```sh
make test
```

Java 17 or newer is sufficient for these host checks.

| Component | Purpose |
| --- | --- |
| [Pose2](src/main/java/dev/ftc/control/Pose2.java) | Finite field poses with normalized radians |
| [PIDController](src/main/java/dev/ftc/control/PIDController.java) | Bounded PID, derivative filtering, conditional-integration anti-windup |
| [EncoderOdometry](src/main/java/dev/ftc/control/EncoderOdometry.java) | Three tracking-wheel deltas with an SE(2) integration step |
| [WaypointFollower](src/main/java/dev/ftc/control/WaypointFollower.java) | Position/heading tolerances, field-to-robot commands, segment timeouts, cancellation |

Coordinates use x forward, y left, and counterclockwise radians. Encoder scale, track width, and lateral-wheel offset must use consistent physical units. The first encoder observation establishes a baseline and does not move the pose. `reset()` supplies a known pose and encoder baseline explicitly.

The waypoint controller returns normalized forward/strafe/turn requests. Its gains are starting constants for simulation, not robot-calibrated values. It stops on completion, timeout, or cancellation. A hardware adapter must read sensors, provide monotonic bounded time steps, apply motor conventions, and enforce robot-specific limits.

Host checks cover straight travel, lateral travel, rotation-offset compensation, PID saturation, waypoint completion, and timeout behavior. Physical-robot validation remains required before deployment.
