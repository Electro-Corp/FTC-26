package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fieldmodeling.DistanceCurve;
import org.firstinspires.ftc.teamcode.fieldmodeling.DistanceDataPoint;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Calibration TeleOp for building a (distance → shooter speed) curve.
 *
 * Workflow:
 *   1. Drive the robot somewhere on the field where you want a calibration point.
 *      The Limelight must see a goal AprilTag (set the right pipeline first —
 *      RIGHT BUMPER on gamepad1 cycles BLUE/RED/OBELISK).
 *   2. Optionally hold gamepad1.x to auto-aim at the tag (drives tx toward 0).
 *      Distance is angle-independent (computed from the tag's 3D pose), so this
 *      is only a convenience for getting consistent shots — it is NOT required
 *      to log a sample.
 *   3. Spin up the shooter (gamepad2.right_trigger) and nudge the speed with
 *      gamepad2.x (+5) and gamepad2.a (-5) until shots land where you want.
 *   4. Press gamepad1.start to log a (distance, speed) sample. The OpMode
 *      averages 20 frames before writing so each sample is stable. The file is
 *      /sdcard/LogParams-Distance.txt and is read by DistanceCurve.
 *
 * This OpMode mirrors DataLoggerTeleOp for full robot functionality (drive,
 * intake, shooter spin/shoot/reverse, manual three-position firing, PID tuning
 * via FTC Dashboard) but logs distance-keyed samples instead of pose-keyed ones.
 */
@Config
@TeleOp(name = "Limelight Calibration TeleOp")
public class LimelightCalibrationTeleOp extends LinearOpMode {

    // ---- FTC Dashboard tunables ----
    /** Shooter PIDF; live-tunable from FTC Dashboard. */
    public static PIDFCoefficients pid = new PIDFCoefficients(12, 0.35, 1, 11);

    /** Auto-aim proportional gain. Lower = gentler, higher = snappier (but oscillates). */
    public static double AUTO_AIM_KP = 0.5;

    /** How many Limelight frames to average per logged sample. */
    public static int CALIBRATION_FRAMES = 20;

    /** Hard timeout in ms while waiting for {@link #CALIBRATION_FRAMES} frames. */
    public static long CALIBRATION_TIMEOUT_MS = 750;

    // ---- Hardware ----
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    private ColorSensors colorSensors;
    private Intake intake;
    private Shooter shooter;
    private Limelight limelight;

    private DistanceCurve curve;

    private final ElapsedTime runtime = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // ---- Edge-detection latches (so a held button only triggers once) ----
    private boolean speedUpHeld = false;
    private boolean speedDownHeld = false;
    private boolean pipelineHeld = false;

    /** Last toast-style message to surface in telemetry after a log action. */
    private String logStatus = "no samples logged yet";

    /**
     * Initialise drive motors, sensors, the shooter subsystem, the Limelight
     * (starting on the BLUE pipeline), and load any existing calibration curve
     * from disk so newly-logged points append rather than overwrite.
     */
    private void initHardware() {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Match the directions used by every other TeleOp in this codebase.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        colorSensors = new ColorSensors(hardwareMap);
        // readColorsOnce = false → reads colors live, matching DataLoggerTeleOp.
        shooter = new Shooter(hardwareMap, colorSensors, false);
        intake = new Intake(hardwareMap, shooter);

        // Calibration is most often done against a goal AprilTag — start on
        // BLUE; gamepad1.right_bumper cycles to RED / OBELISK.
        limelight = new Limelight(hardwareMap, Limelight.PipelineSwitcher.BLUE);

        // Load existing samples so today's points add to yesterday's curve.
        curve = DistanceCurve.read();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            updateDriveMotors();
            readGamepad();

            // Push live PID coefficients from Dashboard into the shooter motors.
            shooter.setPID(pid);

            // ---- Telemetry ----
            telemetry.addData("Shooter State", shooter.getState());
            telemetry.addData("LOADED", "%s %s %s",
                    colorSensors.readLeftColor(),
                    colorSensors.readMidColor(),
                    colorSensors.readRightColor());
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addData("Target Vel (SPINNER_SPEED_NEAR)", shooter.SPINNER_SPEED_NEAR);

            telemetry.addLine("================ LIMELIGHT =================");
            telemetry.addData("Pipeline", limelight.getPipeline());
            if (limelight.hasTarget()) {
                telemetry.addData("tx (deg)", "%.2f", limelight.getTx());
                telemetry.addData("ty (deg)", "%.2f", limelight.getTy());
                telemetry.addData("ta (%)",   "%.2f", limelight.getTa());
                telemetry.addData("Distance (in)", "%.2f", limelight.getDistance());
            } else {
                telemetry.addLine("No target visible");
            }

            telemetry.addLine("================ CALIBRATION ===============");
            telemetry.addData("Curve points", curve.size());
            telemetry.addData("Last log", logStatus);

            // Mirror everything important to FTC Dashboard charts so we can
            // graph distance vs speed live during a tuning session.
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter Vel", shooter.getVelocity());
            packet.put("Target Vel", -(shooter.SPINNER_SPEED_NEAR));
            packet.put("Left shooter Vel", shooter.getLeftVelocity());
            packet.put("Right shooter Vel", shooter.getRightVelocity());
            if (limelight.hasTarget()) {
                packet.put("LL distance (in)", limelight.getDistance());
                packet.put("LL tx", limelight.getTx());
            }
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }
    }

    /**
     * Mecanum drive with the same controls as DataLoggerTeleOp:
     *   - left stick: translation
     *   - right stick X: yaw
     *   - dpad: low-speed nudges
     *   - bumpers: yaw nudges
     *   - A: half-speed crawl
     * Adds: hold gamepad1.x for auto-aim, which adds a tx-driven yaw correction
     * on top of whatever the driver is commanding.
     */
    private void updateDriveMotors() {
        double axial = 0, lateral = 0, yaw = 0;

        if (Math.abs(gamepad1.left_stick_y) > 0.1)  axial = -gamepad1.left_stick_y;
        if (Math.abs(gamepad1.left_stick_x) > 0.1)  lateral = gamepad1.left_stick_x;
        if (Math.abs(gamepad1.right_stick_x) > 0.1) yaw = gamepad1.right_stick_x;

        if (gamepad1.dpad_up)    axial   += 0.3;
        if (gamepad1.dpad_down)  axial   -= 0.3;
        if (gamepad1.dpad_left)  lateral -= 0.3;
        if (gamepad1.dpad_right) lateral += 0.3;

        if (gamepad1.right_bumper) yaw -= 0.3 * -1;
        if (gamepad1.left_bumper)  yaw += 0.3 * -1;

        // Auto-aim: while held, the Limelight contributes additional yaw to drive
        // tx → 0. The driver's yaw stick still works on top of this so they can
        // override or assist the alignment.
        if (gamepad1.x && limelight.hasTarget()) {
            yaw += limelight.getYawCorrection(AUTO_AIM_KP);
        }

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalise so no single wheel exceeds 1.0 commanded power.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Half-speed mode for fine positioning while sampling.
        if (gamepad1.a) {
            leftFrontPower  *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower   *= 0.5;
            rightBackPower  *= 0.5;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Gamepad mapping mirrors DataLoggerTeleOp where applicable, plus calibration-
     * specific bindings:
     *
     *   gamepad1
     *     start         → log current (distance, speed) sample
     *     right_bumper  → cycle Limelight pipeline (BLUE → RED → OBELISK → BLUE)
     *     x             → hold to auto-aim at the visible AprilTag
     *
     *   gamepad2
     *     left_trigger  → intake forward
     *     left_bumper   → intake reverse
     *     right_trigger → spin up shooter
     *     right_bumper  → wait kickers (no auto-fire — keeps speed isolated)
     *     b             → reverse flywheels (clear jam)
     *     x             → SPINNER_SPEED_NEAR += 5  (was ±1 in DataLoggerTeleOp)
     *     a             → SPINNER_SPEED_NEAR -= 5
     *     dpad_left     → fire left kicker
     *     dpad_up       → fire mid kicker
     *     dpad_right    → fire right kicker
     *     y             → stop shooter
     */
    private void readGamepad() {
        // ---- Intake ----
        if (gamepad2.left_trigger >= 0.2) {
            intake.setSpeed(-gamepad2.left_trigger);
            intake.go();
        } else if (gamepad2.left_bumper) {
            intake.reverse();
        } else {
            intake.stop();
        }

        // ---- Shooter ----
        boolean fast = false;

        // right_bumper holds kickers in the wait position without changing the
        // motor speed. DataLoggerTeleOp uses this same "isolation" pattern so we
        // can tune speed without the shooter cycling kickers on every press.
        if (gamepad2.right_bumper) {
            shooter.kickersWait();
        }
        if (gamepad2.right_trigger > 0.2) {
            shooter.spinUp(fast);
        }
        if (gamepad2.b) {
            shooter.reverse(fast);
        }

        // Speed nudge: ±5 ticks/s per press, edge-triggered so a held button
        // doesn't ramp the value. Negative because SPINNER_SPEED_NEAR is stored
        // as a negative number (the shooter motors are wired to spin negatively).
        if (gamepad2.x && !speedUpHeld) {
            shooter.SPINNER_SPEED_NEAR -= 5; // more negative → faster
            speedUpHeld = true;
        } else if (!gamepad2.x) {
            speedUpHeld = false;
        }

        if (gamepad2.a && !speedDownHeld) {
            shooter.SPINNER_SPEED_NEAR += 5; // less negative → slower
            speedDownHeld = true;
        } else if (!gamepad2.a) {
            speedDownHeld = false;
        }

        // Manual three-position firing.
        if (gamepad2.dpad_left) {
            shooter.setShootSpecific(true, false, false);
            if (fast) shooter.shootFar(); else shooter.shootNear();
        }
        if (gamepad2.dpad_up) {
            shooter.setShootSpecific(false, true, false);
            if (fast) shooter.shootFar(); else shooter.shootNear();
        }
        if (gamepad2.dpad_right) {
            shooter.setShootSpecific(false, false, true);
            if (fast) shooter.shootFar(); else shooter.shootNear();
        }
        if (gamepad2.y) {
            shooter.stopShoot();
        }

        shooter.update();

        // ---- Pipeline cycle (gamepad1.right_bumper is also used by drive for
        //      yaw nudging; we add an edge-triggered cycle here that fires only
        //      on press). Drive's yaw nudge is fine to coexist — pressing the
        //      bumper briefly nudges yaw + cycles the pipeline; sustained hold
        //      keeps yawing without spamming pipeline switches. ----
        if (gamepad1.right_bumper && !pipelineHeld) {
            limelight.switchPipeline(limelight.getPipeline().next());
            pipelineHeld = true;
        } else if (!gamepad1.right_bumper) {
            pipelineHeld = false;
        }

        // ---- Log a calibration sample ----
        if (gamepad1.startWasPressed()) {
            logCurrentSample();
        }
    }

    /**
     * Capture an averaged Limelight sample at the robot's current position and
     * pair it with the current SPINNER_SPEED_NEAR. The pair is appended to the
     * in-memory curve and immediately persisted to /sdcard/LogParams-Distance.txt.
     */
    private void logCurrentSample() {
        Limelight.Sample sample = limelight.sampleAveraged(
                CALIBRATION_FRAMES, CALIBRATION_TIMEOUT_MS);

        if (sample == null) {
            logStatus = "FAILED — no valid Limelight frames";
            return;
        }

        double speed = shooter.SPINNER_SPEED_NEAR;
        curve.addPoint(new DistanceDataPoint(sample.distance, speed));
        curve.write();

        logStatus = String.format(
                "logged d=%.2f in, speed=%.0f (avg %d frames)",
                sample.distance, speed, sample.frames);
    }
}
