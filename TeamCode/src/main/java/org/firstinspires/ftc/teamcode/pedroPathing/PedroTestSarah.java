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
        DRIVE_STARTPOS_SHOOT_POS,
        LOOK_AT_APRILTAG,
        SHOOT_PRELOAD,
        GRAB_MORE_ARTIFACTS_ONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.9052132701422, 123.50710900473929, Math.toRadians(36));
    private final Pose aprilTagPose = new Pose(87.651, 87.974, Math.toRadians(95));
    private final Pose shootPose = new Pose(99.65876777251184, 98.82464454976305, Math.toRadians(36)); //TODO how to just turn without moving
    private final Pose turnToGrabOne = new Pose(94.521327014218, 83.75355450236967, Math.toRadians(0));
    private final Pose grabOneArtifact = new Pose(129.5734597156398, 84.12322274881517, Math.toRadians(0));

    private PathChain driveStartPosShootPos, driveAprilTagShootPos, driveGrabArtifactOne;

    public void buildPaths(){
        //put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, aprilTagPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), aprilTagPose.getHeading())
                .build();
        driveAprilTagShootPos = follower.pathBuilder()
                .addPath(new BezierLine(aprilTagPose, shootPose))
                .setLinearHeadingInterpolation(aprilTagPose.getHeading(), shootPose.getHeading())
                .build();
        driveGrabArtifactOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, turnToGrabOne))
                .setLinearHeadingInterpolation(shootPose.getHeading(), turnToGrabOne.getHeading())
                .addPath(new BezierLine(turnToGrabOne, grabOneArtifact))
                .build(); //test, idk if the two paths will work or if I need to add another setlinerHeadingInterpolaiton thingy or create two seperate paths
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.LOOK_AT_APRILTAG); //reset timer and make new state
                break;
            case LOOK_AT_APRILTAG:
                //check is follower done it's path?
                //check that 3 seconds has elapsed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Done Path 1");
                    follower.followPath(driveAprilTagShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                    //transition to next state
                }
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    telemetry.addLine("done with Pose 2");
                    follower.followPath(driveGrabArtifactOne, true);
                    setPathState(PathState.GRAB_MORE_ARTIFACTS_ONE);
                    //transition to next state
                }
                break;
            case GRAB_MORE_ARTIFACTS_ONE:
                if(!follower.isBusy()){
                    telemetry.addLine("done with Pose 3");
                }
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
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
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
