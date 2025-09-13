package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="ChinmayOpMode", group="TeleOp")
public class ChinmayOpMode extends LinearOpMode {
    private int speed = 0;

    private DcMotorEx leftFront = null;
    @Override
    public void runOpMode() throws InterruptedException {
        TestBrain tBrain = new TestBrain(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            AprilTagDetection tag = tBrain.closestTag();


            if(tag != null && tag.ftcPose != null)
                telemetry.addData("Distance", tag.ftcPose.range);
            else
                telemetry.addData("Not found", ":(");
            telemetry.update();
        }
    }
}
