package org.firstinspires.ftc.teamcode.opsmodes.auto;

import android.graphics.Point;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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

    private Thread shooterThread;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();

        waitForStart();

        TrajectoryActionBuilder initTurn = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-18, -18))
                .turn(ang(-45));
        runTrajectory(initTurn);

        shooter.shootNear();

        waitForShooter();

        intake.go();

        initTurn = drive.actionBuilder(drive.localizer.getPose())
                .turn(ang(180))
                .strafeTo(new Vector2d(-18 - 9, -18))
                .strafeTo(new Vector2d(-18 - 9, 0));
        runTrajectory(initTurn);

        intake.stop();

        initTurn = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-18 - 9, -5))
                .turn(ang(-180));
        runTrajectory(initTurn);

        align(id);

        shooter.shootNear();

        waitForShooter();

        shooter.stopShooterThread();
    }

    // Litearlly becuase im lazy
    double ang(double a){
        return Math.toRadians(a);
    }

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooterThread = new Thread(shooter);
        shooterThread.start();
    }

    private void waitForShooter(){
        // Block until shooter is done shooting or force stopped
        while(shooter.getState() != Shooter.ShooterState.STOPPED && !isStopRequested()){
            telemetry.addLine("Waiting for shooter to be done..");
            telemetry.addData("Speed", shooter.getVelocity());
            telemetry.addData("State", shooter.getState());
            telemetry.addData("Shooter Thread is alive", shooterThread.isAlive());
            telemetry.update();
        }
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
        telemetry.addLine("Running trajectory");
        telemetry.update();
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }


    protected abstract int getTargetTag();
}
