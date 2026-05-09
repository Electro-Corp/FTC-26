package org.firstinspires.ftc.teamcode.pedroPathing;
public class PedroTest{

}

//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;

//import com.pedropathing.geometry.Pose;


//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.util.Timer;
//
//@Autonomous(name = "Pedro Test")
//public class PedroTest extends OpMode {
//
//    private Paths paths; // ✅ correct
//
//    private Follower follower;
//    private Timer pathTimer, opmodeTimer;
//
//    private final Pose startPose = new Pose(122.000, 125.300, Math.toRadians(35));
//
//    private int pathState;
//
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        paths = new Paths(follower); // ✅ THIS WAS MISSING
//    }
//
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(paths.Path1, true);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path3, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path4, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path5, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path7, true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path8, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path10, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path11, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//}
