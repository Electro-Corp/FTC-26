//package org.firstinspires.ftc.teamcode.opsmodes;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
//
//@TeleOp(name="RoadRunner Test", group="TeleOp")
//public class RoadRunnerTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initPose = new Pose2d(0,0,0);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
//
//        MecanumDrive.PARAMS.maxWheelVel = 20;
//        MecanumDrive.PARAMS.lateralGain = 0;
//        MecanumDrive.PARAMS.headingGain = 5;
//
//        PinpointLocalizer.PARAMS.parYTicks
//
//        TrajectoryActionBuilder trajectory = drive.actionBuilder(initPose)
//                        .lineToX(30)
//                        .turn(Math.PI / 2)
//                        .lineToY(30);
//
//        waitForStart();
//
//        Action trajAction = trajectory.build();
//
//        Actions.runBlocking(trajAction);
//
//        if(isStopRequested()) return;
//
//
//    }
//}
