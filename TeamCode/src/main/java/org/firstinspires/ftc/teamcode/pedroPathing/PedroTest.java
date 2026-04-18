package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous(name = "Pedro Test")
public class PedroTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private final Pose startPose = new Pose(122.000, 125.300, Math.toRadians(35));

    private int pathState;

    private PathChain Path1, Path2, Path3, Path4, Path5, Path7, Path8, Path10, Path11;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(122.000, 125.300), new Pose(96.400, 96.000)))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(105))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96.400, 96.000), new Pose(89.800, 89.300)))
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(49))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.800, 89.300), new Pose(121.400, 83.500)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(121.400, 83.500), new Pose(87.000, 85.500)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(49))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.000, 85.500),
                        new Pose(95.479, 55.399),
                        new Pose(129.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(129.000, 59.000), new Pose(90.000, 89.400)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(49))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(90.000, 89.400),
                        new Pose(85.005, 43.368),
                        new Pose(119.400, 36.100)))
                .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(180))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.400, 36.100), new Pose(90.000, 89.100)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(49))
                .build();

        Path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.000, 89.100), new Pose(120.100, 12.500)))
                .setLinearHeadingInterpolation(Math.toRadians(49), Math.toRadians(180))
                .build();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path7, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path8, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path10, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(Path11, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
