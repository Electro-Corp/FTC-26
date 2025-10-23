package org.firstinspires.ftc.teamcode.opsmodes.auto;

import android.graphics.Point;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public abstract class AutoRoot extends LinearOpMode {
    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private Intake intake;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();

        waitForStart();

        TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(18);
        runTrajectory(trajectory);

        align(id);

        shooter.shootThreeFar();

        trajectory = drive.actionBuilder(drive.localizer.getPose())
                .turn(Math.PI / 2);
        runTrajectory(trajectory);

        intake.go();

        trajectory = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(18)
                .turn(Math.toRadians(-135));
        runTrajectory(trajectory);

        align(id);

        shooter.shootThreeFar();
    }

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    private void align(int tagId){
        AprilTagDetection tag = tBrain.getTagID(tagId); // Only Red tag right now
        if (tag != null) {
            AprilTagPoseFtc tagPose = tag.ftcPose;

            TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                    .turn(Math.toRadians(tagPose.bearing));

            runTrajectory(trajectory);
        }
    }

    private void runTrajectory(TrajectoryActionBuilder t){
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }


    protected abstract int getTargetTag();
}
