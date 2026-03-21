# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build the project
./gradlew assembleDebug

# Install to connected robot (Control Hub or phone via USB/Wi-Fi)
./gradlew installDebug
```

Deployment is done via Android Studio to the robot over USB or Wi-Fi Direct. There are no unit tests — FTC robot code runs on hardware only.

FTC Dashboard is accessible at `http://192.168.43.1:8080` when connected to the robot's Wi-Fi network. The Pedro Pathing web browser panel is at `http://192.168.43.1:8001`.

## Project Structure

All team-written code lives in:
```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

Key packages:
- `opsmodes/` — TeleOp and autonomous OpModes registered with the FTC app
- `opsmodes/auto/` — Autonomous OpModes (Red/Blue, Near/Far variants)
- `subsystems/` — Hardware abstractions (Shooter, Intake, ColorSensors, Kickers)
- `roadrunner/` — RoadRunner motion planning library integration
- `pedroPathing/` — Pedro Pathing library integration (newer, in-progress)
- `camera/` — AprilTag and Limelight vision
- `fieldmodeling/` — Field state map for distance-based shooter speed lookup
- `sam/` — Experimental Actions-based autonomous approach

## Architecture

### OpMode Hierarchy

**TeleOp**: `MainTeleOp` (abstract) → `RED_TeleOp` / `BLUE_TeleOp`
- `GetSideMultiplier()` returns `+1` for Red, `-1` for Blue, used to mirror coordinates
- On init, reads `/sdcard/end.json` (written by auto) to restore the robot's starting pose

**Autonomous**: `AutoRoot` (abstract) → `RED_Auto_Near`, `RED_Auto_Far`, `BLUE_Auto_Near`, `BLUE_Auto_Far`
- Abstract methods: `getInvert()`, `getTargetTag()`, `getXOffset()`, `isNear()`
- Runs a secondary telemetry thread (implements `Runnable`) alongside the main trajectory thread
- On completion, writes final pose to `/sdcard/end.json` for TeleOp handoff

### Drive / Localization

- **RoadRunner v1** (`roadrunner/MecanumDrive.java`): Main motion planning. Uses `PinpointLocalizer` wrapping the GoBilda Pinpoint odometry pod (hardware name: `"pinpoint"`).
- **Pedro Pathing** (`pedroPathing/Constants.java`): Alternative path follower, also uses the Pinpoint device. Not yet used in competition OpModes.
- Drive motor hardware names: `leftFront`, `rightFront`, `leftBack`, `rightBack`

### Shooter Subsystem (`subsystems/Shooter.java`)

State machine with states: `STOPPED → WAITING_FOR_SPIN_UP → SHOOTING → WAIT_FOR_KICKER → STOPPED`.

- Dual flywheel motors (`shooterLeft`, `shooterRight`) run at negative velocity (target ~-1300 ticks/s near, ~-7000 far)
- Three kicker servos (`lKick`, `mKick`, `rKick`) fire individual balls
- Two dam servos (`leftDam`, `rightDam`) gate ball loading
- Custom PID loop in `PIDControl()` (Kp=-0.03 by default); also supports FTC PIDF coefficients
- Implements `Runnable` — in auto it runs in a background thread via `shooterThread`
- `readColorsOnce=true` in auto (reads colors at init), `false` in teleop (live reads)

### Vision

- `AprilTagBrain` / `TestBrain` (`camera/`): Wraps FTC's AprilTag pipeline. Tags 21–23 are the "obelisk" target markers read in auto to determine the scoring pattern.
- `Limelight3A` used in `LimelightAlign` for fiducial-based turning; hardware name `"limelight"`.

### Field Modeling (`fieldmodeling/`)

`DataLogger` reads `/sdcard/LogParams-FINAL.txt` (a JSON file of field positions → shooting speed/heading) into a `FieldDataPoints` map. Auto and TeleOp use `fieldMap.getStateAtPose(pose)` to look up the correct shooter speed and firing heading for the robot's current position.
