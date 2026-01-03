package org.firstinspires.ftc.teamcode.sam;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Auto with Actions")
public class AutoOpsModeWithActions extends LinearOpMode {
    TestBrain tBrain = null;

    private Intake intake;
    private AutoShooter shooter;
    private ColorSensors colorSensors;


    private DriveActions driveActions;

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        driveActions = new DriveActions(hardwareMap);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new AutoShooter(hardwareMap, colorSensors, tBrain);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        telemetry.addData("Pose", driveActions.getCurrentPoseString());
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                shooter.spinUp(false),
                driveActions.moveToReadObelisk(),
                shooter.readObelisk()
                ));

        //Launch initial balls
        telemetry.addData("Pose", driveActions.getCurrentPoseString());
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToLaunchLocation(),
                shooter.readBallColors(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                shooter.stop(),
                intake.goAction()));

        //Gather row 1 balls
        telemetry.addData("Pose",  driveActions.getCurrentPoseString());
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToRowOfBalls1()));

        //Fire row 1 balls
        telemetry.addData("Pose",  driveActions.getCurrentPoseString());
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToLaunchLocation(),
                shooter.readBallColors(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                shooter.fireNextBall()
        ));
    }
}
