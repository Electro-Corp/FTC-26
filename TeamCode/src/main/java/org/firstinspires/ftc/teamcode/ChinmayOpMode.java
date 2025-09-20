package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name="ChinmayOpMode", group="TeleOp")
public class ChinmayOpMode extends LinearOpMode {
    private int speed = 0;

    private DcMotorEx leftFront = null;
    @Override
    public void runOpMode() throws InterruptedException {
        TestBrain tBrain = new TestBrain(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            AprilTagPoseFtc tag = tBrain.getClosestTag();

            telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size());
            if(tag != null) {
                telemetry.addData("Range", tag.range);
                telemetry.addData("Bearing", tag.bearing);
            }else{
                telemetry.addLine("No Pose avaliable");
            }
            telemetry.update();
        }
    }
}
