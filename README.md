# 2025 Spectrum Reefscape Robot

WPILib / command-based robot code for the 2025 FRC *Reefscape* season. Hardware is driven through CTRE Phoenix 6 (Talon FX motors, CANcoders, swerve), with PathPlanner for autonomous and a MapleSim-backed swerve simulation.

This README focuses on the **state-machine architecture** that ties the robot together: a top-level `Superstructure` coordinator that issues `WantedState`s to each subsystem, and per-subsystem state machines that translate those requests into hardware setpoints.

---

## Repository layout

```
src/main/java/frc/
├── reefscape/                   Field geometry, AprilTag helpers, scoring zones
│   ├── FieldConstants.java
│   ├── FieldHelpers.java
│   └── Zones.java
├── robot/
│   ├── Main.java                JVM entry point
│   ├── Robot.java               TimedRobot — wires subsystems + button bindings
│   ├── RobotSim.java            Simulation Mechanism2d views
│   ├── auton/Auton.java         Autonomous chooser / PathPlanner integration
│   ├── configs/OM2025.java      Per-robot config bundle
│   ├── constants/Constants.java Scoring offsets, reef-face/tag maps
│   ├── operator/Operator.java   Operator gamepad
│   ├── pilot/Pilot.java         Driver gamepad
│   └── subsystems/
│       ├── Superstructure.java  ← top-level coordinator
│       ├── claw/Claw.java
│       ├── elevator/Elevator.java
│       ├── shoulder/Shoulder.java
│       ├── swerve/Swerve.java
│       └── vision/Vision.java
└── spectrumLib/                 Shared mechanism / sim / swerve / gamepad lib
```

The `spectrumLib` package contains the reusable `Mechanism` base class that each subsystem extends, simulation helpers (`LinearSim`, `ArmSim`, `RollerSim`), gamepad wrappers, and the MapleSim swerve drivetrain integration.

---

## Architecture: layered state machines

Every output-bearing subsystem follows the same pattern:

1. A `WantedState` enum — what the rest of the code is asking for.
2. A `SystemState` enum — what the subsystem has actually decided to do this loop (may differ from `WantedState` due to local guards, e.g. sensor readings).
3. A `periodic()` that runs every 20 ms:
   ```
   systemState = handleStateTransition();   // WantedState → SystemState
   applyStates();                           // SystemState → motor outputs
   ```
4. A `setWantedState(WantedState)` setter — the only external way to drive the subsystem.

The [Superstructure](src/main/java/frc/robot/subsystems/Superstructure.java) sits one level above and follows the same pattern with `WantedSuperState` / `CurrentSuperState`. Inside its `applyStates()`, it issues `setWantedState(...)` calls into Swerve, Elevator, Shoulder, and Claw. This means **subsystems never talk to each other** — they only obey the Superstructure.

Driver inputs flow:

```
Pilot gamepad button
   └─ Superstructure.setWantedSuperState(WantedSuperState.X)
        └─ Superstructure.periodic()
             ├─ Swerve.setWantedState(...)
             ├─ Elevator.setWantedState(...)
             ├─ Shoulder.setWantedState(...)
             └─ Claw.setWantedState(...)
```

---

## Superstructure

File: [src/main/java/frc/robot/subsystems/Superstructure.java](src/main/java/frc/robot/subsystems/Superstructure.java)

### `WantedSuperState`

Idle / lifecycle: `HOME`, `STOPPED`, `DEFAULT_STATE`, `IDLE_EMPTY`, `IDLE_ALGAE`, `IDLE_CORAL`.

Algae handling: `ALGAE_GROUND_INTAKE`, `ALGAE_L2_INTAKE`, `ALGAE_L3_INTAKE`, `ALGAE_NET_PREP`, `ALGAE_NET_SCORE`, `ALGAE_PROCESSOR_SCORE`.

Coral handling: `CORAL_INTAKE_FLOOR`, `CORAL_PREPARE_HANDOFF`, `CORAL_RELEASE_HANDOFF`, `CORAL_L1_PREP`, `CORAL_L1_SCORE`, `CORAL_L{2,3,4}_{LEFT,RIGHT}_SCORE`.

Climbing: `CLIMING_APPROACH`, `CLIMBING_HANG`, `CLIMBING_LOCK`.

### Transition + apply

[Superstructure.java:128](src/main/java/frc/robot/subsystems/Superstructure.java#L128) (`handStateTransitions`) is mostly a passthrough — most wanted states map directly onto the matching current state. `DEFAULT_STATE` resolves based on what the claw is holding: `IDLE_CORAL` if `hasCoral()`, `IDLE_ALGAE` if `hasAlgae()`, otherwise `IDLE_EMPTY`.

[Superstructure.java:191](src/main/java/frc/robot/subsystems/Superstructure.java#L191) (`applyStates`) dispatches to per-state helpers. Each helper composes a coherent robot pose:

| Current state | Swerve | Elevator | Shoulder | Claw |
| --- | --- | --- | --- | --- |
| `STOPPED` | `TELEOP_DRIVE` | `STOPPED` | `STOPPED` | `OFF` |
| `IDLE_EMPTY` | `TELEOP_DRIVE` | `HOME` | `HOME` | `OFF` |
| `IDLE_CORAL` | `TELEOP_DRIVE` | `STOWED_CORAL` | `STOWED_CORAL` | `COLLECT_CORAL` |
| `IDLE_ALGAE` | `TELEOP_DRIVE` | `STOWED_ALGAE` | `STOWED_ALGAE` | `COLLECT_ALGAE` |
| `CORAL_L{2,3,4}_*_SCORE` | `DRIVE_TO_POINT` (computed reef pose) | `CORAL_L{2,3,4}_LINEUP` → `_RELEASE` | matching `LINEUP` → `RELEASE` | `COLLECT_CORAL` → `EJECT_CORAL` |
| `ALGAE_GROUND_INTAKE` | — | `ALGAE_GROUND_INTAKE` | `ALGAE_GROUND_INTAKE` | `COLLECT_ALGAE` |
| `ALGAE_L{2,3}_INTAKE` | `DRIVE_TO_POINT` (reef algae pose) | `ALGAE_L{2,3}_INTAKE` | `ALGAE_L{2,3}_INTAKE` | `COLLECT_ALGAE` |
| `ALGAE_NET_PREP` | `ROTATION_LOCK` to 0° + slow translation | `ALGAE_NET` | `ALGAE_NET` | `COLLECT_ALGAE` |
| `ALGAE_NET_SCORE` | `ROTATION_LOCK` to 0° + slow translation | `ALGAE_NET` | `ALGAE_NET` | `EJECT_ALGAE` |

Two recurring patterns:

- **Two-phase scoring.** The score-coral helpers ([scoreL4Coral](src/main/java/frc/robot/subsystems/Superstructure.java#L274), [scoreL3Coral](src/main/java/frc/robot/subsystems/Superstructure.java#L292), [scoreL2Coral](src/main/java/frc/robot/subsystems/Superstructure.java#L310)) first command everything to `*_LINEUP` and call [driveToCoralScoringPose](src/main/java/frc/robot/subsystems/Superstructure.java#L376). Once [isReadyToEject](src/main/java/frc/robot/subsystems/Superstructure.java#L369) is true (drive at translation setpoint, heading within 2°, elevator + shoulder at setpoint), they advance to `*_RELEASE` and `EJECT_CORAL`.
- **Velocity scaling when the arm is high.** During net prep/score, the Superstructure caps teleop translation at 10% (`ARM_HIGH_TELEOP_TRANSLATION_COEFFICIENT`) to keep the robot stable with the elevator extended.

### Driver bindings

[Robot.configureBindings](src/main/java/frc/robot/Robot.java#L155) wires the pilot bumpers/triggers/face buttons through [Superstructure.configureButtonBinding](src/main/java/frc/robot/subsystems/Superstructure.java#L415). That helper picks one of three target super-states based on what the claw is holding — `(hasCoralCondition, hasAlgaeCondition, noPieceCondition)` — and releases back to `DEFAULT_STATE` on button release. Pilot D-pad left/right manually flip the "holding coral / holding algae" sensor flags ([Claw.forceSetHolding…](src/main/java/frc/robot/subsystems/claw/Claw.java#L203)).

---

## Subsystem state machines

### Swerve

File: [src/main/java/frc/robot/subsystems/swerve/Swerve.java](src/main/java/frc/robot/subsystems/swerve/Swerve.java)

Extends CTRE `SwerveDrivetrain<TalonFX, TalonFX, CANcoder>`. Four states:

| State | Behavior |
| --- | --- |
| `IDLE` | No `setControl` call — modules coast in their last request. |
| `TELEOP_DRIVE` | Field-centric drive from pilot joystick inputs, scaled by `teleopVelocityCoefficient`. Includes alliance flipping and a small skew-compensation term. |
| `ROTATION_LOCK` | Pilot drives translation, heading PID drives to `desiredRotation` (`SwerveRequest.FieldCentricFacingAngle`). |
| `DRIVE_TO_POINT` | Closed-loop drive to `desiredPoseForDriveToPoint`. A `PIDController(3,0,0)` on remaining distance produces speed, capped at `getSpeedAt12Volts()`, plus a 2% static-friction kick when distance ≥ 0.5″. Heading is held at the target pose's rotation. |

Convenience setters [setTargetRotation](src/main/java/frc/robot/subsystems/swerve/Swerve.java#L275) and [setDesiredPoseForDriveToPoint](src/main/java/frc/robot/subsystems/swerve/Swerve.java#L285) update the goal and switch the wanted state in one call. [isAtDriveToPointSetpoint](src/main/java/frc/robot/subsystems/swerve/Swerve.java#L298) returns true when the translation error drops below 1″ (`TRANSLATION_ERROR_MARGIN_METERS`).

### Elevator

File: [src/main/java/frc/robot/subsystems/elevator/Elevator.java](src/main/java/frc/robot/subsystems/elevator/Elevator.java)

Two TalonFX motors (`ElevatorFront` + `ElevatorRear` follower, aligned), Motion Magic position control on a rotational setpoint. Soft limits at `minRotations = 0.1`, `maxRotations = 34`.

`handleStateTransition` ([Elevator.java:177](src/main/java/frc/robot/subsystems/elevator/Elevator.java#L177)) is a 1-to-1 mapping. `applyStates` picks a target rotation per state:

| State | Target rotations |
| --- | --- |
| `HOME`, `STOWED_CORAL`, `STOWED_ALGAE`, `CORAL_L1_RELEASE`, `CLIMBING` | 0.5 |
| `STOPPED` | `stop()` (motor neutralized) |
| `ALGAE_PROCESSOR` | 0.0 |
| `CORAL_L1_LINEUP` | 1.5 |
| `CORAL_L2_LINEUP` / `_RELEASE` | 7.3 |
| `ALGAE_GROUND_INTAKE` | 10.5 |
| `ALGAE_L2_INTAKE` | 14.4 |
| `CORAL_L3_LINEUP` / `_RELEASE` | 17.2 |
| `ALGAE_L3_INTAKE` | 24.9 |
| `HANDOFF` | 27.4 |
| `PRE_CORAL_HANDOFF` | 31.0 |
| `CORAL_L4_LINEUP` / `_RELEASE` | 33.0 |
| `ALGAE_NET` | 33.966 |

[isAtSetpoint](src/main/java/frc/robot/subsystems/elevator/Elevator.java#L326) uses `triggerTolerance = 1.15` rotations.

**Arm-collision guard** ([Elevator.java:291-295](src/main/java/frc/robot/subsystems/elevator/Elevator.java#L291-L295)): before issuing the Motion Magic setpoint, if the commanded position would drop below 10 rotations while the elevator is currently above 10 *and* `Superstructure.armLow()` is true, the target is clamped to 10.5. This blocks descents through the shoulder's swing volume while the arm is tipped past horizontal.

### Shoulder

File: [src/main/java/frc/robot/subsystems/shoulder/Shoulder.java](src/main/java/frc/robot/subsystems/shoulder/Shoulder.java)

Single TalonFX with an optional fused CANcoder. Motion Magic on rotations, with a 90° offset so 0° is "vertical up." A 101.25:1 sensor-to-mechanism ratio means software angles map directly to physical shoulder degrees.

Position table (degrees, before the +90° offset):

| State | Target degrees |
| --- | --- |
| `HOME`, `STOWED_CORAL`, `STOWED_ALGAE` | 0 |
| `STOPPED` | `stop()` |
| `CORAL_GROUND_INTAKE` | 4 |
| `ALGAE_NET` | −19 |
| `CORAL_L4_LINEUP` | −55 |
| `CORAL_L3_LINEUP` | −56.2 |
| `CORAL_L2_LINEUP` | −58 |
| `CORAL_L2_RELEASE`, `CORAL_L3_RELEASE` | −77.8 |
| `CORAL_L4_RELEASE` | −82.4 |
| `ALGAE_L2_INTAKE`, `ALGAE_L3_INTAKE` | −88 |
| `CLIMB_PREP` | −100 |
| `CORAL_INTAKE_LOLIPOP` | −110 |
| `ALGAE_GROUND_INTAKE` | −125 |
| `ALGAE_PROCESSOR` | −143.877 |
| `PRE_CORAL_HANDOFF`, `HANDOFF` | −180 |

[checkMoveOverTop](src/main/java/frc/robot/subsystems/shoulder/Shoulder.java#L376) wraps targets ±360° so the shoulder takes the short path past vertical when transitioning between front and back hemispheres. `triggerTolerance = 3°`.

[shoulderLow](src/main/java/frc/robot/subsystems/shoulder/Shoulder.java#L376) returns true when the post-offset shoulder angle is below −90° or above +90° — i.e., the arm is tipped past horizontal in either direction. It's exposed up through [Superstructure.armLow](src/main/java/frc/robot/subsystems/Superstructure.java#L407) and consumed by the Elevator's arm-collision guard.

### Claw

File: [src/main/java/frc/robot/subsystems/claw/Claw.java](src/main/java/frc/robot/subsystems/claw/Claw.java)

A single roller motor running torque-current FOC control. Unlike the other subsystems, the claw's `handleStateTransition` ([Claw.java:107](src/main/java/frc/robot/subsystems/claw/Claw.java#L107)) is **not** a passthrough — it auto-promotes collect → hold based on sensor flags:

- `COLLECT_ALGAE` + `hasAlgae()` ⇒ `HOLD_ALGAE`
- `COLLECT_ALGAE` + not holding ⇒ `COLLECT_ALGAE`
- `COLLECT_CORAL` + `hasCoral()` ⇒ `HOLD_CORAL`
- `COLLECT_CORAL` + not holding ⇒ `COLLECT_CORAL`
- everything else is a passthrough.

Torque-current targets per system state:

| State | Torque current (A) |
| --- | --- |
| `COLLECT_ALGAE`, `HOLD_ALGAE` | +150 |
| `EJECT_ALGAE` | −200 (clears the algae-held flag) |
| `EJECT_ALGAE_PROCESSOR` | −50 (clears the algae-held flag) |
| `COLLECT_CORAL` | +30 |
| `HOLD_CORAL` | +5 |
| `EJECT_CORAL` | −25 (clears the coral-held flag) |
| `OFF` | `stop()` |

The "has game piece" sensors are currently driver-flagged booleans (`forceHoldingCoralFlag`, `forceHoldingAlgaeFlag`) toggled from the D-pad rather than read from hardware.

---

## Robot lifecycle

[Robot.java](src/main/java/frc/robot/Robot.java) is a `TimedRobot`. The constructor:

1. Starts telemetry.
2. Loads a per-robot config via `Rio.id` (currently only `OM2025`).
3. Constructs the gamepads, then each subsystem with a 0.1 s delay between CAN-bus initializations.
4. Constructs the Superstructure with references to the four motion subsystems.
5. Calls `configureBindings()` to wire pilot buttons → Superstructure state requests.

`robotPeriodic()` runs the WPILib `CommandScheduler` (which calls every subsystem's `periodic()`) and pushes the swerve pose into `Field2d`.

In disabled mode, [Robot.disabledPeriodic](src/main/java/frc/robot/Robot.java#L271) watches the auton chooser, loads PathPlanner paths for the selected routine, flips/mirrors them for the alliance and starting side, and warms up the first path so the robot doesn't stutter at auton start.

---

## Build & run

Standard WPILib Gradle project:

```powershell
.\gradlew build              # compile + spotbugs
.\gradlew simulateJava       # run desktop sim (uses MapleSim-backed swerve)
.\gradlew deploy             # deploy to the RoboRIO
```

PathPlanner autos live under `src/main/deploy/pathplanner/`. The dashboard chooser is wired up in [Auton.java](src/main/java/frc/robot/auton/Auton.java).
