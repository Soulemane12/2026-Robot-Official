---
name: Intake Tuning Session
description: Intake pivot and roller subsystems are built and wired, currently need real-robot position tuning
type: project
---

Intake subsystem (pivot + roller) was just physically built. The code is complete and wired up but the position constants need to be found on the real robot.

**Why:** The placeholder values in Constants.java don't match the real robot's physical range yet.

**How to apply:** When continuing this work, the user needs to run the robot, jog the intake, and record encoder values from SmartDashboard.

## Current state of the code

### Subsystems
- `IntakePivotSubsystem.java` — TalonFX on CAN ID 31, MotionMagic control
- `IntakeRollerSubsystem.java` — TalonFX on CAN ID 32, simple VoltageOut

### Placeholder constants (Constants.java IntakeConstants — ALL NEED REAL VALUES)
- `STOW = 5.0` rotations
- `INTAKE_POSITION = 98` rotations
- `EXTENDED = 94` rotations
- `JOG_VOLTAGE = 2.3` volts
- `ROLLER_VOLTAGE = 8.0` volts

### Soft limits in IntakePivotSubsystem.java (lines 54-60) — ALSO NEED REAL VALUES
- Lower limit: `0.67` rotations
- Upper limit: `101` rotations

### Current operator bindings (RobotContainer.java)
- `operator.leftBumper()` hold → jog pivot OUT (+2.3V)
- `operator.rightBumper()` hold → **TEMPORARY** jog pivot IN (-2.3V) — added this session to find positions, remove after tuning (normally this button tracks AprilTag with turret)
- `operator.rightTrigger()` → go to STOW
- `operator.leftTrigger()` → go to EXTENDED
- `operator.Y()` → go to INTAKE_POSITION
- `operator.X()` → toggle intake roller

### Tuning process (in progress, not yet done)
1. Deploy to robot
2. Open SmartDashboard, watch `Intake/PosRot`
3. Jog intake to each physical position, record `Intake/PosRot` value
4. Update Constants.java with real values
5. Update soft limits in IntakePivotSubsystem.java lines 54-60
6. Remove temporary rightBumper binding, restore turret track
7. Tune PID/MotionMagic gains if needed (kP=2.0, kD=0.02, kS=0.25, kV=0.18)
