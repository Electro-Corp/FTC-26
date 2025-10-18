package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="RoadRunner Test", group="TeleOp")
public class RoadRunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);


        TrajectoryActionBuilder trajectory = drive.actionBuilder(initPose)
                        .turn(Math.PI / 2);
                        //.strafeTo(new Vector2d(18, 00))
                        //.strafeTo(new Vector2d(18, -18))
                        //.strafeTo(new Vector2d(0, -18))
                        //.strafeTo(new Vector2d(0, 0));

        waitForStart();

        Action trajAction = trajectory.build();

        Actions.runBlocking(trajAction);

        if(isStopRequested()) return;


    }
}
