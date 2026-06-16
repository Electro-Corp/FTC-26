# Limelight Distance Calibration TeleOp

A standalone TeleOp for building a `(distance → shooter speed)` calibration
curve so the robot can automatically pick the right flywheel speed for any
shooting distance.

This document captures both **what was built** and **why each decision was
made**, so the team and future Claude sessions can extend it without
re-deriving the design.

---

## TL;DR

- New OpMode: `opsmodes/LimelightCalibrationTeleOp.java` (registered as
  "Limelight Calibration TeleOp" on the driver station).
- Mirrors `DataLoggerTeleOp` for full robot functionality (drive, intake,
  shooter spin/shoot/reverse, manual three-position firing, live PID tuning
  via FTC Dashboard).
- Adds Limelight telemetry (`tx`, `ty`, `ta`, distance), pipeline cycling,
  optional auto-aim, ±5 speed nudge, and a 20-frame averaged sample log.
- Samples are written to **`/sdcard/LogParams-Distance.txt`** in a new
  `(distance, speed)` schema, separate from the existing
  `/sdcard/LogParams-FINAL.txt` pose-keyed file.
- New helpers: `subsystems/Limelight` (rewritten in-place, additive),
  `fieldmodeling/DistanceDataPoint`, `fieldmodeling/DistanceCurve`.

---

## Files Touched

| File | Status | What changed |
|---|---|---|
| `opsmodes/LimelightCalibrationTeleOp.java` | **new** | The calibration TeleOp. Maintains two parallel curves (single-shot and three-shot); `gamepad2.a` toggles the active mode. `gamepad1.start` logs to whichever file the active mode points at; `gamepad1.back` (3 s hold) clears the active curve. The old `gamepad2.a` "−5 speed" handler is commented out — uncomment on a free button if needed. Mode resets to SINGLE on every OpMode start. |
| `opsmodes/MainTeleOp.java` | modified | Initializes a `Limelight` on the alliance pipeline and loads BOTH the single-shot and three-shot `DistanceCurve`s. Replaces the hard-coded `SPINNER_SPEED_NEAR = -1300` with a curve lookup gated by a driver toggle (`gamepad1.left_stick_button`, default ON). `gamepad2.a` toggles between SINGLE and THREE-SHOT firing modes — the existing `shootThreeSpeed` flag drives this; `computeTargetSpeed()` picks the right curve. Removed the legacy `-= 60` heuristic (three-shot adjustment now lives in calibration data). Falls back to -1300 when the active curve is empty / no target / toggle off. Calls `shooter.retargetVelocity()` every loop so live distance changes are pushed to the flywheels continuously. Telemetry shows the speed source, fire mode, both curve sizes, current Limelight distance, and target speed. |
| `subsystems/Shooter.java` | modified | Added `retargetVelocity()` which re-applies `SPINNER_SPEED_NEAR` to the flywheels when the shooter is in a forward-spinning state (`SPIN_UP_HOLD`, `WAITING_FOR_SPIN_UP`, `SHOOTING`). No-op in `STOPPED` / `REVERSE` so it doesn't fight other commands. |
| `subsystems/Limelight.java` | modified | Existing `getID()`, `PipelineSwitcher`, `update()`, `switchPipeline()` preserved verbatim. `getTx`/`getTy`/`getTa` rewritten to read fresh frames (NaN on no target). Added `getDistance()`, `sampleAveraged()`, `getYawCorrection()`, `hasTarget()`, `nextPipeline()`, a `Sample` data class, and a `(HardwareMap, PipelineSwitcher)` constructor that bypasses the auto-pipeline-switching logic. |
| `fieldmodeling/DistanceDataPoint.java` | **new** | Plain `(distance, speed)` struct + `toJSON()`. |
| `fieldmodeling/DistanceCurve.java` | **new** | Reads/writes a calibration file. Linear interpolation between sorted points, clamped at endpoints. `clear()` for the wipe button. Each instance carries its own `filePath` so single-shot and three-shot curves can coexist. Default path is `LogParams-Distance.txt` (single-shot); `THREE_SHOT_FILE_PATH` constant points at `LogParams-Distance-ThreeShot.txt`. New `read(String filePath)` overload. |

`fieldmodeling/DataLogger`, `DataPoint`, and `FieldDataPoints` are untouched —
the existing pose-keyed logging continues to work unchanged.

---

## How to Use It

### Logging a calibration session

1. Power on the robot. The Limelight should be configured with three
   pipelines on the web UI:
   - **0 = BLUE** alliance goal AprilTag
   - **1 = RED** alliance goal AprilTag
   - **2 = OBELISK** (pattern tags 21/22/23)
2. Select **"Limelight Calibration TeleOp"** on the driver station and start.
3. The OpMode boots on pipeline **BLUE**. Press `gamepad1.right_bumper` to
   cycle to the pipeline that matches the goal you're calibrating against.
4. Drive to a position on the field where you want a calibration point.
   Confirm "Distance (in)" appears in telemetry — if it doesn't, the
   Limelight doesn't see a tag.
5. Optionally hold `gamepad1.x` to auto-aim. Distance is angle-independent
   (3D pose, not `ty` trig), so this is purely for shot consistency, not
   for measurement accuracy.
6. Spin up the shooter with `gamepad2.right_trigger`. Adjust speed with
   `gamepad2.x` (+5) and `gamepad2.a` (-5) until shots land in the goal.
7. Press `gamepad1.start` to log the sample. The OpMode averages 20
   Limelight frames (≈200 ms) and writes the `(distance, speed)` pair to
   `/sdcard/LogParams-Distance.txt`.
8. Repeat at varied distances (every ~6 in is a good density). Aim for at
   least 6–10 points across the shooting range.

### Clearing the curve

Hold `gamepad1.back` for **3 seconds**. A countdown appears in telemetry.
Releasing before 3 s cancels. After the clear, both the in-memory curve
and the file on disk are empty.

### Multiple sessions

Logged samples **accumulate across sessions** — `DistanceCurve.read()` runs
in `initHardware()` and loads any pre-existing file before more points are
added. There is no automatic deduplication; if you log at d=50 today and
again tomorrow, both points stay (sorted by distance, both contribute to
the local interpolation). Use the clear button to start fresh.

### Reading the curve in other OpModes

`MainTeleOp` is already wired up to consume the curve. The pattern it uses
is the recommended one for any future OpMode that wants distance-based
speed:

```java
// At init time, exactly once:
distanceCurve = DistanceCurve.read();
limelight = new Limelight(hardwareMap, allianceLane);

// Each loop:
double speed;
if (!distanceSpeedEnabled
        || distanceCurve.size() == 0
        || !limelight.hasTarget()
        || Double.isNaN(limelight.getDistance())) {
    speed = FALLBACK_NEAR_SPEED;
} else {
    speed = distanceCurve.getSpeedAtDistance(limelight.getDistance());
}
shooter.SPINNER_SPEED_NEAR = speed;
```

`getSpeedAtDistance` returns the linear interpolation between the two
neighboring points, clamping at the endpoints if `distance` is outside the
sampled range. Returns 0 if the curve is empty (which is why the explicit
empty-curve check above falls back to a known-good static value instead).

### MainTeleOp integration specifics

`MainTeleOp` is the canonical match TeleOp (`RED_TeleOp` and `BLUE_TeleOp`
extend it). The integration adds:

- **Init.** `Limelight` is constructed on the alliance pipeline —
  `GetSideMultiplier()` returns +1 for red, -1 for blue, so red boots on
  pipeline `RED`, blue boots on `BLUE`. `DistanceCurve.read()` is called
  once at init.
- **Toggle.** `gamepad1.left_stick_button` flips
  `distanceSpeedEnabled` (default ON), edge-detected so a held button
  doesn't oscillate.
- **Per-loop selection.** `computeTargetSpeed()` runs every loop with the
  4-step fallback chain above and writes the result to
  `shooter.SPINNER_SPEED_NEAR`. The pre-existing `shootThreeSpeed` -60
  offset still applies on top of whichever source was chosen.
- **Continuous flywheel update.** Immediately after writing
  `SPINNER_SPEED_NEAR`, the loop calls `shooter.retargetVelocity()` which
  re-applies the new value to both flywheel motors. Without this the
  motors latch whatever velocity was set at the most recent
  `spinUp()`/`shootNear()` call, so distance-based updates would stay
  invisible until the operator triggered a new shoot command — exactly
  the bug we hit on first test (first shot from close-up locked in a low
  speed; moving to the far end didn't speed the wheels up). The
  retarget method is a no-op when the shooter is `STOPPED` / `REVERSE`
  so it can't fight other commands.
- **Telemetry.** A "Speed source" line shows one of
  `CURVE` / `TOGGLE OFF (fallback)` / `EMPTY CURVE (fallback)` / `NO TARGET (fallback)`
  so the driver can tell at a glance whether vision is actually steering
  the shooter. The current Limelight distance and the chosen target speed
  are also shown.
- **Backwards-compatible.** If the legacy `rrEnabled` flag is ever flipped
  back on, the original pose-keyed `fieldMap.getStateAtPose()` path still
  wins. The new path runs in the `else` branch.

---

## Controls

### gamepad1 (driver)

| Input | Action |
|---|---|
| Left stick | Translation |
| Right stick X | Yaw |
| D-pad | Low-speed nudges |
| Bumpers | Yaw nudges (existing convention) |
| `a` | Half-speed crawl |
| `x` (hold) | Auto-aim at visible AprilTag (additive yaw) |
| `right_bumper` | Cycle Limelight pipeline (BLUE → RED → OBELISK) |
| `start` | Log current `(distance, speed)` sample |
| `back` (hold 3 s) | Clear entire curve and rewrite the file empty |

> Note: `right_bumper` does double duty — drive uses it for a yaw nudge while
> held, and the calibration code cycles the pipeline on the leading edge of
> the press. A short tap cycles the pipeline; a sustained hold yaws without
> spamming pipeline switches.

### gamepad2 (operator)

Same as `DataLoggerTeleOp`, **except** the speed-nudge step size:

| Input | Action |
|---|---|
| `left_trigger` | Intake forward (variable speed) |
| `left_bumper` | Intake reverse |
| `right_trigger` | Spin up shooter |
| `right_bumper` | Hold kickers in wait position (isolation for tuning) |
| `b` | Reverse flywheels (clear jam) |
| `x` | `SPINNER_SPEED_NEAR -= 5` (faster — value is negative) |
| `a` | `SPINNER_SPEED_NEAR += 5` (slower) |
| `dpad_left` | Fire left kicker |
| `dpad_up` | Fire mid kicker |
| `dpad_right` | Fire right kicker |
| `y` | Stop shooter |

`DataLoggerTeleOp` nudges by ±1 — the user requested ±5 for this OpMode.

---

## Design Decisions

This section is the why-it-works-the-way-it-does, captured during the design
conversation so future contributors don't have to re-litigate the same
trade-offs.

### Why a separate file instead of extending `LogParams-FINAL.txt`?

The existing pose-keyed file stores `(posX, posY, heading, speed)` for the
RoadRunner localizer. The calibration curve is a different shape —
`(distance, speed)` — and needs different interpolation (1D linear vs the
existing 3-nearest-neighbors weighted by inverse distance in 2D pose space).
Forcing the new schema into the old file would either (a) shoehorn distance
into `posX` and confuse anyone reading the file, or (b) require a schema
migration on every existing TeleOp. A new file keeps the existing behavior
intact and the calibration data self-describing.

### Why the Limelight 3D pose for distance instead of the classic `ty` trig?

Three options were considered:

1. **`ty` + camera height + tag height + mount angle (classic Limelight trig).**
   Accurate when robot is square to the tag (tx ≈ 0). Errors grow with off-axis
   angle (~1.5% at tx=10°, ~6% at tx=20°, ~13% at tx=30°). Requires three
   physical constants to be measured and kept in sync with mounting changes.

2. **FTC AprilTag `ftcPose.range`.** Calibrated 3D range, but uses the
   webcam (`Webcam 1`), not the Limelight. Doesn't fit a "Limelight
   calibration" workflow.

3. **Limelight 3A `getTargetPoseRobotSpace()` (chosen).** Returns the tag's
   full 3D pose in robot coordinates. Distance = `hypot(x, z)` is correct
   regardless of viewing angle. Slightly noisier than the trig formula at
   long range, mitigated by averaging.

The deciding factor was the user's note: **in the far shooting zone the robot
physically cannot face the AprilTag head-on.** That kills option 1 — its
errors are largest exactly where calibration is most needed. Option 3 stays
correct in that regime, and the noise concern is solved by averaging.

### Why average 20 frames per sample?

The Limelight's 3D pose estimator is noisier than its raw `tx`/`ty`/`ta`
because it solves PnP from corner detections. Single-frame readings can wobble
±0.5 in at typical distances. Averaging 20 frames (~200 ms at 100 Hz) cuts
that noise by ~√20 ≈ 4.5×, bringing per-sample error well below the
`±5` speed nudge granularity.

The averaging loop also dedups frames by reference so a tight calling loop
can't double-count the same frame between Limelight publishes — important
because we sleep 10 ms between reads but the camera publishes at 100 Hz and
the timing isn't strictly aligned.

### Why hold-to-clear instead of a single button?

A 3-second hold prevents accidental wipes. Calibration sessions are
expensive (10+ samples per side, with shooting between each). A bumped
button shouldn't destroy the dataset. The countdown in telemetry gives
the operator visible feedback that the timer is running.

### Why is the auto-aim "additive" instead of overriding driver yaw?

The auto-aim correction is added to whatever yaw the driver is commanding,
not substituted for it. This means:

- The driver always retains override authority.
- The operator can hold `x` while the driver coarsely points at the tag,
  and the correction gently zeros tx without a "jolt" handoff.
- If the Limelight loses the target mid-correction, `getYawCorrection()`
  returns 0 (not NaN) so the robot doesn't spin or jerk.

Distance measurement is angle-independent, so auto-aim is a convenience for
shot consistency, not a prerequisite for logging a sample.

### Why preserve the existing `Limelight` API instead of rewriting?

The original `Limelight` subsystem has an auto-pipeline-switching
`update()` method (BLUE/RED in TeleOp, OBELISK reads in auto), plus a
`getID()` method called by other code that wasn't yet built. Rewriting
those would have broken existing/planned callers. Instead:

- The two-arg `Limelight(boolean isTeleOp, boolean isRed)` constructor
  stays untouched.
- A new `Limelight(HardwareMap, PipelineSwitcher)` constructor was added
  for the calibration TeleOp, which wants to pick its own pipeline and
  bypass the auto-switching.
- `getTx`/`getTy`/`getTa` were rewritten to read a fresh frame each call
  and return `NaN` on no-target instead of the original `1000` sentinel.
  This was the user's explicit choice: the new behavior composes cleanly
  with `Math.isNaN()` checks elsewhere, where 1000 would have silently
  flowed into controllers as a giant error.

### Why `±5` step size and not `±1`?

`DataLoggerTeleOp` uses ±1, which is too fine for shooter speed tuning —
real-world MoE on flywheel velocity is ~10–30 ticks/s, so ±1 nudges are
noise. `±5` is large enough to feel a difference per press but small enough
to converge without overshooting. The user specified this directly.

### Why not also expose ±50 / "fast nudge"?

Not requested, and the current control surface is already crowded. If
needed later, the obvious mapping would be `gamepad2.dpad_up_left` or
`gamepad2.dpad_up_right` for ±50, but that conflicts with the existing
manual-fire bindings. A cleaner addition would be a modifier — e.g.
`gamepad2.left_stick_button` held = 10× step size on x/a — but that's
extra UX complexity for unclear benefit. Defer until the team finds the
±5 step actually slow during a session.

### Why `gamepad1.start` for logging instead of `gamepad2.something`?

Matches `DataLoggerTeleOp`'s convention so the muscle memory transfers.
Also keeps logging on the driver gamepad — driver positions the robot,
driver decides when the sample is "good", driver presses start. The
operator is busy with shooter speed.

### Open question: should the OpMode also display "predicted speed" from current curve?

A potentially-useful addition: while logging, show
`curve.getSpeedAtDistance(currentDistance)` so the operator can compare
the current `SPINNER_SPEED_NEAR` against what the existing curve would
predict. Useful for spotting outliers and stale data. **Not implemented.**
Could be added in `runOpMode()`'s telemetry block in ~3 lines.

### Open question: should samples be timestamped or session-tagged?

Currently every sample is just `(distance, speed)`. No session ID, no
timestamp, no notes ("after intake redesign", "with new flywheels"). For a
single robot revision this is fine, but if the team iterates on hardware,
old data may need to be discarded — and right now that's an all-or-nothing
clear. Two ways to address if it becomes a problem:

1. Add `timestamp` and `sessionId` fields to `DistanceDataPoint`. Backwards
   compatible with `DistanceCurve` (extra fields are ignored on read).
2. Auto-roll the file on each session start, leaving dated archives. Simpler
   to implement, harder to merge across sessions.

Defer until needed.

---

## Implementation Notes

### Pipeline constants live on the Limelight

The OpMode imports `Limelight.PipelineSwitcher.BLUE` etc. from the existing
enum. If new pipelines are added (e.g. a separate "long range" tuning
pipeline), update the enum and the cycle order via `next()` in
`subsystems/Limelight.java` — no changes needed in the TeleOp.

### Distance unit

`DistanceCurve` and `Limelight.getDistance()` both use **inches**. The
Limelight conversion is done explicitly via
`pos.toUnit(DistanceUnit.INCH)` inside `Limelight.distanceFrom()`. If the
team prefers cm/mm, change in one place.

### Shooter speed sign

`SPINNER_SPEED_NEAR` is stored **negative** because the shooter motors are
wired to spin negatively at full power. The nudge logic reflects this:
`gamepad2.x` makes the value more negative (faster), `gamepad2.a` makes
it less negative (slower). Telemetry displays the raw value (negative),
which can be confusing — consider showing `Math.abs(speed)` as
"target speed" if that becomes a problem.

### File path is hard-coded

`DistanceCurve.FILE_PATH = "/sdcard/LogParams-Distance.txt"` is a
public-static-final and is referenced nowhere else. To rename, change it
in one place. To support multiple curves (e.g. one per shooter
configuration), add a `String filePath` field and constructor param to
`DistanceCurve`.

---

## Suggested Future Work

In rough order of value-to-effort:

1. **Use the curve in autonomous.** `MainTeleOp` now consumes the curve
   (see "MainTeleOp integration specifics" above). The 4 auto OpModes
   (`AutoRoot` and its alliance/distance variants) still set
   `SPINNER_SPEED_NEAR` to hard-coded values like `-1220` and `-1600`.
   Wire the same `Limelight` + `DistanceCurve` lookup in there, with the
   same fallback chain, so auto benefits from calibration too.
2. **Show predicted speed in calibration telemetry.** While logging,
   display `curve.getSpeedAtDistance(currentDistance)` next to the manual
   `SPINNER_SPEED_NEAR` so the operator can compare the new sample
   against what the existing curve would have predicted. Useful for
   spotting outliers and stale data.
3. **Add an "expected scoring zone" overlay** — annotate each curve
   point with the field zone it was logged in (NEAR / FAR / arbitrary).
   Useful for debugging when the curve has weird discontinuities.
4. **Replace linear interpolation with a smoothing spline** if the
   physics gets nonlinear (e.g. the relationship is closer to
   `speed ∝ √distance`). Linear is a fine starting point and produces
   debuggable, predictable values.
5. **Validate samples before accepting them.** Reject if `tx > X`° (robot
   too off-axis, suspect data) or `ta < Y`% (tag too small, distance
   estimate noisy).
6. **Add a "remove last sample" button** for fixing fat-finger logs
   without nuking the whole curve.
7. **Auto-skip duplicate samples.** If a new point is within 1 in of an
   existing one, replace rather than append.
8. **Hot-reload the curve mid-match (debug only).** Re-read the file on a
   button press so a fresh calibration session can be tested without
   restarting the OpMode. Disable in competition.

---

## Session Conversation Summary

The full design was negotiated in conversation. Key turning points:

1. User requested an OpMode like `DataLoggerTeleOp` for distance/speed
   calibration with full robot functionality.
2. Asked clarifying questions (8 of them); user answered each.
3. Initial recommendation was the `ty` trig formula. User asked whether
   it accounts for off-axis viewing — it doesn't.
4. Auto-aim-then-sample suggested as a workaround. User pointed out that
   in the far zone, facing the tag head-on is **physically impossible**.
   This invalidated the trig approach entirely.
5. Switched to Limelight 3D pose with averaging. User approved, requested
   averaging.
6. Implemented files: `Limelight` (preserved + extended), `DistanceCurve`,
   `DistanceDataPoint`, `LimelightCalibrationTeleOp`.
7. User asked about cross-session behavior. File is appended, not
   overwritten. User requested a clear button.
8. Initial 1-second hold; user requested 3 seconds. Done.

The key lesson for future sessions: **always probe the physical
constraints before recommending a vision strategy**. The far-zone
constraint flipped the entire approach from "pick a trig formula and
calibrate the constants" to "use the camera's full 3D pose."
