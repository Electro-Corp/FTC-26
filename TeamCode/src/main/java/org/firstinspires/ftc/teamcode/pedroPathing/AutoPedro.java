package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.opsmodes.auto.Pattern;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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

    private TestBrain tBrain;
    private Shooter shooter;
    private ColorSensors colorSensors;
    private Intake intake;
    private Thread shooterThread;

    private Pattern pattern;
    private int shotCount = 0;
    private boolean[] fired = new boolean[3];
    private boolean shootPhaseInitialized = false;

    private static final PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0.3, 0.5, 12.5);
    private static final double NEAR_SHOOT_SPEED = -1220;

    public enum PathState {
        //START POSITION_END POSITION
        INIT,
        DRIVE_TO_APRILTAG,
        WAIT_AT_APRILTAG,
        TURN_TO_SHOOT,
        WAIT_FOR_TURN,
        SHOOT_PRELOADED,
        DRIVE_TO_ROW_ONE,
        DRIVE_COLLECT_ROW_ONE,
        DRIVE_SHOOT_ROW_ONE,
        SHOOT_ROW_ONE,
        DRIVE_TO_ROW_TWO,
        DRIVE_COLLECT_ROW_TWO,
        DRIVE_SHOOT_ROW_TWO,
        SHOOT_ROW_TWO,
        PARK
    }

    PathState pathState;

    // All poses are in relationship to RED. They are then mirrored if they are for the blue side
    private final Pose startPose = pose(124, 123.5, Math.toRadians(36));
    private final Pose aprilTagPose = pose(95, 95, Math.toRadians(120));
    private final Pose shootPose = pose(95, 95, Math.toRadians(45));
    private final Pose rowOnePose = pose(95, 84, Math.toRadians(180));
    private final Pose collectRowOnePose = pose(125, 84, Math.toRadians(180));
    private final Pose rowTwoPose = pose(93, 57, Math.toRadians(180));
    private final Pose collectRowTwoPose = pose(130, 57, Math.toRadians(180));
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
//                if (pattern == null) {
//                    for (int i = 21; i <= 23; i++) {
//                        if (tBrain.isIdOnScreen(i)) {
//                            pattern = Pattern.fromNum(i);
//                            break;
//                        }
//                    }
//                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    if (pattern == null) {
//                        pattern = Pattern.fromNum(new Random().nextInt(3) + 21);
//                    }
                    setPathState(PathState.TURN_TO_SHOOT);
                }
                break;
            case TURN_TO_SHOOT:
                follower.turnTo(shootPose.getHeading());
                setPathState(PathState.WAIT_FOR_TURN);
                break;
            case WAIT_FOR_TURN:
                if (!follower.isTurning() && pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(PathState.SHOOT_PRELOADED);
                }
                break;
            case SHOOT_PRELOADED:
                if (pathTimer.getElapsedTimeSeconds() > 1 /*tryShootThree()*/) {
                    follower.followPath(driveToRowOne, true);
//                    intake.go();
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
//                    shooter.spinUp(false, false);
                    setPathState(PathState.DRIVE_SHOOT_ROW_ONE);
                }
                break;
            case DRIVE_SHOOT_ROW_ONE:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_ROW_ONE);
                }
                break;
            case SHOOT_ROW_ONE:
                if (pathTimer.getElapsedTimeSeconds() > 1 /*tryShootThree()*/) {
                    follower.followPath(driveToRowTwo, true);
//                    intake.go();
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
//                    shooter.spinUp(false, false);
                    setPathState(PathState.DRIVE_SHOOT_ROW_TWO);
                }
                break;
            case DRIVE_SHOOT_ROW_TWO:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_ROW_TWO);
                }
                break;
            case SHOOT_ROW_TWO:
                if (pathTimer.getElapsedTimeSeconds() > 1 /*tryShootThree()*/) {
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

    // Non-blocking version of AutoRoot.shootThree(). Returns true once all 3 balls have fired.
    private boolean tryShootThree() {
        if (!shootPhaseInitialized) {
            shooter.spinUp(false, true);
            shooter.setDamDown();
            shooter.updateLoadedColors();
            shotCount = 0;
            fired = new boolean[3];
            shootPhaseInitialized = true;
            return false;
        }
        if (shotCount < 3) {
            Shooter.ShooterState s = shooter.getState();
            if (s == Shooter.ShooterState.STOPPED || s == Shooter.ShooterState.SPIN_UP_HOLD) {
                fireNextFromPattern();
                shotCount++;
            }
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
                if (!fired[i] && shooter.getLoadedColors()[i] == targetColor) {
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

    @Override
    public void init() {
        pathState = PathState.INIT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

//        tBrain = new TestBrain(hardwareMap);
//        colorSensors = new ColorSensors(hardwareMap);
//        shooter = new Shooter(hardwareMap, colorSensors, true);
//        shooter.setPID(SHOOTER_PID);
//        shooter.SPINNER_SPEED_NEAR = NEAR_SHOOT_SPEED;
//        intake = new Intake(hardwareMap);
//
//        shooterThread = new Thread(shooter);
//        shooterThread.start();

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
//        shooter.spinUp(false, false);
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
//        telemetry.addData("pattern", pattern == null ? "null" : pattern.toString());
//        telemetry.addData("shooter state", shooter.getState());
    }

    @Override
    public void stop() {
//        if (shooter != null) {
//            shooter.stopShoot();
//            shooter.stopShooterThread();
//        }
//        if (intake != null) intake.stop();
    }
}
