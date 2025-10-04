package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="RoadRunner Test", group="TeleOp")
public class OdoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);


        TrajectoryActionBuilder trajectory = drive.actionBuilder(initPose)
                        .strafeTo(new Vector2d(10, 0))
                        .strafeTo(new Vector2d(10, 10))
                        .strafeTo(new Vector2d(0, 10))
                        .strafeTo(new Vector2d(0, 0));

        waitForStart();

        Action trajAction = trajectory.build();

        Actions.runBlocking(trajAction);

        if(isStopRequested()) return;


    }
}
