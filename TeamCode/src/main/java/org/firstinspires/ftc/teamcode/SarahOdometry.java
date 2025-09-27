package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class SarahOdometry extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //MecanumDrive drive = new MecanumDrive(new Pose2d())
                //.strafeRight(10)
                //.forward(5)
                //.build();

        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectory(myTrajectory);
    }
}
