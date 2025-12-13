package org.firstinspires.ftc.teamcode.sam;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Calibrate Actions")
public class CalibrateActionsOpsMode extends LinearOpMode {
    Pose2d initPose = new Pose2d(0, 0, 0);

    private MecanumDrive drive;

    private void initHardware() {
        drive = new MecanumDrive(hardwareMap, initPose);
        initPose = new Pose2d(0,0,0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        Actions.runBlocking(new SequentialAction(

                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(12,0), 0).build(),

                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(new Vector2d(12,12), 0).build()

        ));
    }


}
