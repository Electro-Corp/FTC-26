package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.camera.AprilTagBrain;
import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="traingle")
public class Triangulate extends LinearOpMode {
    TestBrain testBrain;
    @Override
    public void runOpMode() throws InterruptedException {
        testBrain = new TestBrain(hardwareMap);

        while(!isStarted()){
            AprilTagBrain.DesertPoint desertPoint = testBrain.getCurrentPositionRelativeToCenterOfBackWall();
            telemetry.addData("Tags on screen", testBrain.getVisibleTags().size());
            telemetry.addData("X", desertPoint.x);
            telemetry.addData("Y", desertPoint.y);
            AprilTagDetection tag = testBrain.getTagID(24);
            if(tag != null) {
                telemetry.addData("Bearing to target", tag.ftcPose.bearing);
                telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", (tag.ftcPose.y * Math.cos(tag.ftcPose.bearing)), (tag.ftcPose.y * Math.sin(tag.ftcPose.bearing)), tag.ftcPose.z);
            }
            telemetry.update();
        }
    }
}
