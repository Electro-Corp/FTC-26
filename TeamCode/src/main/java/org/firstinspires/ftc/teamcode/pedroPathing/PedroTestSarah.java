package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous (name="Most Amazingest Auto Everrrrrrr", group = "Autonomous")
public class PedroTestSarah extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

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

    private final Pose startPose = new Pose(123.9052132701422, 123.50710900473929, Math.toRadians(36));
    private final Pose aprilTagPose = new Pose(87.651, 87.974, Math.toRadians(95));
    private final Pose shootPose = new Pose(87.651, 87.974, Math.toRadians(45));
    private final Pose rowOnePose = new Pose(94.521327014218, 83.75355450236967, Math.toRadians(180));
    private final Pose collectRowOnePose = new Pose(129.5734597156398, 84.12322274881517, Math.toRadians(180));
    private final Pose rowTwoPose = new Pose(92.81516587, 59.20379146, Math.toRadians(180));
    private final Pose collectRowTwoPose = new Pose(129.573459, 59.5450236, Math.toRadians(180));
    private final Pose parkPose = new Pose(90, 120, Math.toRadians(45));

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
                .addPath(new BezierLine(collectRowTwoPose, shootPose))
                .setLinearHeadingInterpolation(collectRowTwoPose.getHeading(), shootPose.getHeading())
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
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.TURN_TO_SHOOT);
                }
                break;
            case TURN_TO_SHOOT:
                follower.turnTo(shootPose.getHeading());
                setPathState(PathState.WAIT_FOR_TURN);
                break;
            case WAIT_FOR_TURN:
                if (!follower.isTurning()) {
                    setPathState(PathState.SHOOT_PRELOADED);
                }
                break;
            case SHOOT_PRELOADED:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(driveToRowOne, true);
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
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_ROW_ONE);
                }
                break;
            case SHOOT_ROW_ONE:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(driveToRowTwo, true);
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
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_ROW_TWO);
                }
                break;
            case SHOOT_ROW_TWO:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
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

    @Override
    public void init() {
        pathState = PathState.INIT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());



    }
}
