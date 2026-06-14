package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.camera.BallColorPipeline;
import org.firstinspires.ftc.teamcode.opsmodes.auto.Pattern;
import java.util.Random;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public abstract class AutoPedro extends OpMode {

    private static final double FIELD_WIDTH = 144.0;

    /** Returns true if this opmode is for the Blue alliance. */
    protected abstract boolean isBlue();

    /**
     * Pose constructor that mirrors across the field's vertical center for Blue.
     * Red uses the literal coordinates; Blue gets x → (144 - x) and heading → (π - heading).
     */
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

    private static final PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0.3, 0.5, 12.5);
    private static final double NEAR_SHOOT_SPEED = -1340;
    private static final double TURN_TIMEOUT_SECONDS = 2.0;
    // Seconds after leaving the collect position before spinning up (lowering the dam).
    // Gives balls time to settle before the dam drops. Tune between 0.5 and 1.0.
    private static final double DAM_LOWER_DELAY_SECONDS = 0.5;
    private static final double SPINUP_SPEED_TOLERANCE = 100;
    private static final double SPINUP_TIMEOUT_SECONDS = 1.5;
    private static final long SPINUP_STABLE_MS = 150;

    public enum PathState {
        //START POSITION_END POSITION
        INIT,
        DRIVE_TO_APRILTAG,
        WAIT_AT_APRILTAG,
        TURN_TO_SHOOT,
        WAIT_FOR_TURN,
        WAIT_FOR_SPINUP,
        SHOOT_PRELOADED,
        DRIVE_TO_ROW_ONE,
        DRIVE_COLLECT_ROW_ONE,
        DRIVE_SHOOT_ROW_ONE,
        ALIGN_ROW_ONE,
        SHOOT_ROW_ONE,
        DRIVE_TO_ROW_TWO,
        DRIVE_COLLECT_ROW_TWO,
        DRIVE_SHOOT_ROW_TWO,
        ALIGN_ROW_TWO,
        SHOOT_ROW_TWO,
        PARK
    }

    PathState pathState;
    private PathState stateAfterSpinup = PathState.SHOOT_PRELOADED;

    // All poses are in relationship to RED. They are then mirrored if they are for the blue side
    private final Pose startPose = pose(124, 123.5, Math.toRadians(36));
    private final Pose aprilTagPose = pose(95, 95, Math.toRadians(120));
    private final Pose shootPose = pose(90, 90, Math.toRadians(40));
    private final Pose rowOnePose = pose(95, 84, Math.toRadians(180));
    private final Pose collectRowOnePose = pose(128, 84, Math.toRadians(180));
    private final Pose rowTwoPose = pose(93, 56, Math.toRadians(180));
    private final Pose collectRowTwoPose = pose(134, 56, Math.toRadians(180));
    private final Pose rowTwoWaypointPose = pose(110, 60, Math.toRadians(180));
    private final Pose parkPose = pose(100, 120, Math.toRadians(45));

    private PathChain driveToAprilTag, driveToRowOne, driveCollectRowOne, driveShootRowOne, driveToRowTwo, driveCollectRowTwo, driveShootRowTwo, drivePark;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveToAprilTag = follower.pathBuilder()
                .addPath(new BezierLine(startPose, aprilTagPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), aprilTagPose.getHeading())
                .build();
        driveToRowOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rowOnePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), rowOnePose.getHeading())
                .build();
        driveCollectRowOne = follower.pathBuilder()
                .addPath(new BezierLine(rowOnePose, collectRowOnePose))
                .setLinearHeadingInterpolation(rowOnePose.getHeading(), collectRowOnePose.getHeading())
                .build();
        driveShootRowOne = follower.pathBuilder()
                .addPath(new BezierLine(collectRowOnePose, shootPose))
                .setLinearHeadingInterpolation(collectRowOnePose.getHeading(), shootPose.getHeading())
                .build();
        driveToRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rowTwoPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), rowTwoPose.getHeading())
                .build();
        driveCollectRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(rowTwoPose, collectRowTwoPose))
                .setLinearHeadingInterpolation(rowTwoPose.getHeading(), collectRowTwoPose.getHeading())
                .build();
        driveShootRowTwo = follower.pathBuilder()
                .addPath(new BezierLine(collectRowTwoPose, rowTwoWaypointPose))
                .setLinearHeadingInterpolation(collectRowTwoPose.getHeading(), rowTwoWaypointPose.getHeading())
                .addPath(new BezierLine(rowTwoWaypointPose, shootPose))
                .setLinearHeadingInterpolation(rowTwoWaypointPose.getHeading(), shootPose.getHeading())
                .build();
        drivePark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState){
            case INIT:
                follower.followPath(driveToAprilTag, true);
                setPathState(PathState.DRIVE_TO_APRILTAG); //reset timer and make new state
                break;
            case DRIVE_TO_APRILTAG:
                if (!follower.isBusy()) {
                    setPathState(PathState.WAIT_AT_APRILTAG);
                }
                break;
            case WAIT_AT_APRILTAG:
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
                    setPathState(PathState.TURN_TO_SHOOT);
                }
                break;
            case TURN_TO_SHOOT:
                follower.turnTo(shootPose.getHeading());
                setPathState(PathState.WAIT_FOR_TURN);
                break;
            case WAIT_FOR_TURN:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
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
                    follower.followPath(driveToRowOne, true);
                    intake.go();
                    setPathState(PathState.DRIVE_TO_ROW_ONE);
                }
                break;
            case DRIVE_TO_ROW_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollectRowOne, false);
                    setPathState(PathState.DRIVE_COLLECT_ROW_ONE);
                }
                break;
            case DRIVE_COLLECT_ROW_ONE:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootRowOne, true);
                    setPathState(PathState.DRIVE_SHOOT_ROW_ONE);
                }
                break;
            case DRIVE_SHOOT_ROW_ONE:
                if (pathTimer.getElapsedTimeSeconds() > DAM_LOWER_DELAY_SECONDS) {
                    shooter.spinUp(false, false);
                }
                if (!follower.isBusy()) {
                    // Stop intake the moment we begin aligning so it doesn't run during
                    // the turn or the shooting phase. It will resume after the shot
                    // sequence completes via the intake.go() call when transitioning to
                    // the next collect path.
                    intake.stop();
                    follower.turnTo(shootPose.getHeading());
                    setPathState(PathState.ALIGN_ROW_ONE);
                }
                break;
            case ALIGN_ROW_ONE:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
                    stateAfterSpinup = PathState.SHOOT_ROW_ONE;
                    setPathState(PathState.WAIT_FOR_SPINUP);
                }
                break;
            case SHOOT_ROW_ONE:
                if (tryShootThree()) {
                    follower.followPath(driveToRowTwo, true);
                    intake.go();
                    setPathState(PathState.DRIVE_TO_ROW_TWO);
                }
                break;
            case DRIVE_TO_ROW_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollectRowTwo, false);
                    setPathState(PathState.DRIVE_COLLECT_ROW_TWO);
                }
                break;
            case DRIVE_COLLECT_ROW_TWO:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootRowTwo, true);
                    setPathState(PathState.DRIVE_SHOOT_ROW_TWO);
                }
                break;
            case DRIVE_SHOOT_ROW_TWO:
                if (pathTimer.getElapsedTimeSeconds() > DAM_LOWER_DELAY_SECONDS) {
                    shooter.spinUp(false, false);
                }
                if (!follower.isBusy()) {
                    // Same intake-stop-on-align pattern as row one. Row two is the
                    // last shooting phase, so intake doesn't need to resume.
                    intake.stop();
                    follower.turnTo(shootPose.getHeading());
                    setPathState(PathState.ALIGN_ROW_TWO);
                }
                break;
            case ALIGN_ROW_TWO:
                if (!follower.isTurning() || pathTimer.getElapsedTimeSeconds() > TURN_TIMEOUT_SECONDS) {
                    stateAfterSpinup = PathState.SHOOT_ROW_TWO;
                    setPathState(PathState.WAIT_FOR_SPINUP);
                }
                break;
            case SHOOT_ROW_TWO:
                if (tryShootThree()) {
                    follower.followPath(drivePark, true);
                    setPathState(PathState.PARK);
                }
                break;
            case PARK:
                if (!follower.isBusy()) {
                    intake.stop();
                    shooter.stopShoot();
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

    private long spinupInRangeStart = -1;

    private boolean isShooterReady() {
        double target = Math.abs(shooter.SPINNER_SPEED_NEAR);
        double current = Math.abs(shooter.getVelocity());
        boolean inRange = current >= target - SPINUP_SPEED_TOLERANCE
                       && current <= target + SPINUP_SPEED_TOLERANCE;
        if (inRange) {
            if (spinupInRangeStart < 0) spinupInRangeStart = System.currentTimeMillis();
            return System.currentTimeMillis() - spinupInRangeStart >= SPINUP_STABLE_MS;
        } else {
            spinupInRangeStart = -1;
            return false;
        }
    }

    // Non-blocking version of AutoRoot.shootThree(). Returns true once all 3 balls have fired.
    private boolean tryShootThree() {
        if (!shootPhaseInitialized) {
            intake.stop();
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
        // Wait for the last ball's kicker to finish its full cycle before stopping
        Shooter.ShooterState s = shooter.getState();
        if (s != Shooter.ShooterState.SPIN_UP_HOLD && s != Shooter.ShooterState.STOPPED) {
            return false;
        }
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
        //shooter.setPID(SHOOTER_PID);
        shooter.SPINNER_SPEED_NEAR = NEAR_SHOOT_SPEED;
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
