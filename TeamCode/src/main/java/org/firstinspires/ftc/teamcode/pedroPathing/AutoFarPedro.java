package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.camera.BallColorPipeline;
import org.firstinspires.ftc.teamcode.opsmodes.auto.Pattern;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Random;

public abstract class AutoFarPedro extends OpMode {

    private static final double FIELD_WIDTH = 144.0;

    protected abstract boolean isBlue();

    protected Pose pose(double x, double y, double headingRadians) {
        if (isBlue()) {
            return new Pose(FIELD_WIDTH - x, y, Math.PI - headingRadians);
        }
        return new Pose(x, y, headingRadians);
    }

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private Limelight limelight;
    private BallColorPipeline ballCam;
    private Shooter shooter;
    private ColorSensors colorSensors;
    private Intake intake;
    private Thread shooterThread;

    private Pattern pattern;
    private int shotCount = 0;
    private boolean[] fired = new boolean[3];
    private boolean shootPhaseInitialized = false;
    private BallColor[] loadedColors = {BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
    private boolean intakeRunning = false;

    private static final PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(20, 0.6, 0.5, 12.5);
    private static final double FAR_SHOOT_SPEED = -1460;
    private static final double TURN_TIMEOUT_SECONDS = 2.0;
    private static final double DAM_LOWER_DELAY_SECONDS = 0.5;
    // How close the flywheel velocity must be to the target before allowing shots.
    private static final double SPINUP_SPEED_TOLERANCE = 50;
    private static final double SPINUP_TIMEOUT_SECONDS = 5.0;

    public enum PathState {
        INIT,
        WAIT_AT_OBELISK,
        DRIVE_TO_SHOOT,
        WAIT_FOR_TURN,
        WAIT_FOR_SPINUP,
        SHOOT_PRELOADED,
        TURN_TO_COLLECT,
        WAIT_FOR_COLLECT_TURN,
        DRIVE_COLLECT,
        DRIVE_SHOOT_ROW_ONE,
        ALIGN_ROW_ONE,
        SHOOT_ROW_ONE,
        PARK
    }

    PathState pathState;
    // Which shoot state to enter after the spinup wait completes.
    private PathState stateAfterSpinup = PathState.SHOOT_PRELOADED;

    // All coordinates are Red-side. Blue mirrors via pose(): x → (144-x), heading → (π-heading).
    private final Pose startPose        = pose(86.5,  9.7,  Math.toRadians(90));
    // Drive destination before the shoot turn — same x,y as shootPose, heading still 90°.
    private final Pose preTurnPose      = pose(86,    16,   Math.toRadians(90));
    // Final shoot heading after the in-place turn.
    private final Pose shootPose        = pose(86,    16,   Math.toRadians(65));
    // Start of the collect drive — same x,y as shootPose but already facing the row (180°).
    private final Pose collectStartPose = pose(86,    16,   Math.toRadians(180));
    private final Pose collectPose      = pose(134.6, 9.5,  Math.toRadians(180));
    private final Pose parkPose         = pose(95,    25,   Math.toRadians(90));

    private PathChain driveToShoot, driveCollect, driveShoot, drivePark;

    private void buildPaths() {
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, preTurnPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preTurnPose.getHeading())
                .build();
        // Robot is already facing 180° after the in-place turn; hold that heading throughout.
        driveCollect = follower.pathBuilder()
                .addPath(new BezierLine(collectStartPose, collectPose))
                .setConstantHeadingInterpolation(collectPose.getHeading())
                .build();
        // Rotate from 180° back to 65° during the return drive.
        driveShoot = follower.pathBuilder()
                .addPath(new BezierLine(collectPose, shootPose))
                .setLinearHeadingInterpolation(collectPose.getHeading(), shootPose.getHeading())
                .build();
        drivePark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case INIT:
                setPathState(PathState.WAIT_AT_OBELISK);
                break;

            case WAIT_AT_OBELISK:
                if (pattern == null) {
                    LLResult llResult = limelight.getLatestResult();
                    if (llResult != null && llResult.isValid()) {
                        for (LLResultTypes.FiducialResult f : llResult.getFiducialResults()) {
                            int id = f.getFiducialId();
                            if (id >= 21 && id <= 23) {
                                pattern = Pattern.fromNum(id);
                                break;
                            }
                        }
                    }
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (pattern == null) {
                        pattern = Pattern.fromNum(new Random().nextInt(3) + 21);
                    }
                    follower.followPath(driveToShoot, true);
                    setPathState(PathState.DRIVE_TO_SHOOT);
                }
                break;

            case DRIVE_TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.turnTo(shootPose.getHeading());
                    setPathState(PathState.WAIT_FOR_TURN);
                }
                break;

            case WAIT_FOR_TURN:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
                    shooter.SPINNER_SPEED_NEAR = FAR_SHOOT_SPEED;
                    stateAfterSpinup = PathState.SHOOT_PRELOADED;
                    setPathState(PathState.WAIT_FOR_SPINUP);
                }
                break;

            case WAIT_FOR_SPINUP:
                if (isShooterReady() || pathTimer.getElapsedTimeSeconds() > SPINUP_TIMEOUT_SECONDS) {
                    setPathState(stateAfterSpinup);
                }
                break;

            case SHOOT_PRELOADED:
                if (tryShootThree()) {
                    setPathState(PathState.TURN_TO_COLLECT);
                }
                break;

            case TURN_TO_COLLECT:
                follower.turnTo(collectStartPose.getHeading());
                setPathState(PathState.WAIT_FOR_COLLECT_TURN);
                break;

            case WAIT_FOR_COLLECT_TURN:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
                    intakeRunning = true;
                    follower.followPath(driveCollect, false);
                    setPathState(PathState.DRIVE_COLLECT);
                }
                break;

            case DRIVE_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(driveShoot, true);
                    setPathState(PathState.DRIVE_SHOOT_ROW_ONE);
                }
                break;

            case DRIVE_SHOOT_ROW_ONE:
                if (pathTimer.getElapsedTimeSeconds() > DAM_LOWER_DELAY_SECONDS) {
                    shooter.spinUp(false, false);
                }
                if (!follower.isBusy()) {
                    follower.turnTo(shootPose.getHeading());
                    setPathState(PathState.ALIGN_ROW_ONE);
                }
                break;

            case ALIGN_ROW_ONE:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
                    shooter.SPINNER_SPEED_NEAR = FAR_SHOOT_SPEED;
                    stateAfterSpinup = PathState.SHOOT_ROW_ONE;
                    setPathState(PathState.WAIT_FOR_SPINUP);
                }
                break;

            case SHOOT_ROW_ONE:
                if (tryShootThree()) {
                    intakeRunning = false;
                    intake.stop();
                    follower.followPath(drivePark, true);
                    setPathState(PathState.PARK);
                }
                break;

            case PARK:
                if (!follower.isBusy()) {
                    telemetry.addLine("parked");
                }
                break;

            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    private boolean isShooterReady() {
        double target = Math.abs(shooter.SPINNER_SPEED_NEAR);
        double current = Math.abs(shooter.getVelocity());
        return current >= target - SPINUP_SPEED_TOLERANCE;
    }

    // Non-blocking shoot loop. Returns true once all 3 balls have fired.
    private boolean tryShootThree() {
        if (!shootPhaseInitialized) {
            shooter.spinUp(false, true);
            shooter.setDamDown();
            loadedColors = ballCam.getDetectedColors();
            shotCount = 0;
            fired = new boolean[3];
            shootPhaseInitialized = true;
            return false;
        }
        if (shotCount < 3) {
            Shooter.ShooterState s = shooter.getState();
            if ((s == Shooter.ShooterState.STOPPED || s == Shooter.ShooterState.SPIN_UP_HOLD)
                    && isShooterReady()) {
                fireNextFromPattern();
                shotCount++;
            }
            return false;
        }
        // Wait for the last kicker cycle to finish before stopping.
        Shooter.ShooterState s = shooter.getState();
        if (s != Shooter.ShooterState.SPIN_UP_HOLD && s != Shooter.ShooterState.STOPPED) {
            return false;
        }
        shooter.stopShoot();
        shooter.kickersWait();
        shooter.setDamUp();
        shootPhaseInitialized = false;
        return true;
    }

    private void fireNextFromPattern() {
        BallColor targetColor = pattern != null ? pattern.getColorAtIndex(shotCount) : null;

        int indexToShoot = -1;
        if (targetColor != null) {
            for (int i = 0; i < 3; i++) {
                if (!fired[i] && loadedColors[i] == targetColor) {
                    indexToShoot = i;
                    break;
                }
            }
        }
        if (indexToShoot == -1) {
            for (int i = 0; i < 3; i++) {
                if (!fired[i]) { indexToShoot = i; break; }
            }
        }
        if (indexToShoot == -1) return;

        switch (indexToShoot) {
            case 0: shooter.setShootSpecific(true, false, false); break;
            case 1: shooter.setShootSpecific(false, true, false); break;
            case 2: shooter.setShootSpecific(false, false, true); break;
        }
        shooter.shootNear();
        fired[indexToShoot] = true;
    }

    private static String colorChar(BallColor c) {
        if (c == BallColor.GREEN)  return "G";
        if (c == BallColor.PURPLE) return "P";
        return "?";
    }

    private static String colorRow(Pattern p) {
        if (p == null) return "? | ? | ?";
        return colorChar(p.getColorAtIndex(0)) + " | "
             + colorChar(p.getColorAtIndex(1)) + " | "
             + colorChar(p.getColorAtIndex(2));
    }

    private static String colorRow(BallColor[] colors) {
        return colorChar(colors[0]) + " | "
             + colorChar(colors[1]) + " | "
             + colorChar(colors[2]);
    }

    @Override
    public void init() {
        pathState = PathState.INIT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        limelight = new Limelight(hardwareMap, Limelight.PipelineSwitcher.OBELISK);
        ballCam = new BallColorPipeline(hardwareMap);
        loadedColors = ballCam.getDetectedColors();
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter(hardwareMap, colorSensors, true);
        shooter.setPID(SHOOTER_PID);
        shooter.SPINNER_SPEED_NEAR = FAR_SHOOT_SPEED;
        intake = new Intake(hardwareMap);

        shooterThread = new Thread(shooter);
        shooterThread.start();

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        shooter.spinUp(false, false);
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        if (intakeRunning) intake.go();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Obelisk", colorRow(pattern));
        telemetry.addData("Loaded ", colorRow(loadedColors));
        telemetry.addData("shooter state", shooter != null ? shooter.getState() : "null");
        telemetry.addData("shooter velocity", shooter != null ? shooter.getVelocity() : "null");
        telemetry.addData("shooter target", shooter != null ? shooter.SPINNER_SPEED_NEAR : "null");
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.stopShoot();
            shooter.stopShooterThread();
        }
        if (intake != null) intake.stop();
        if (ballCam != null) ballCam.close();
        if (limelight != null) limelight.raw().stop();
    }
}
