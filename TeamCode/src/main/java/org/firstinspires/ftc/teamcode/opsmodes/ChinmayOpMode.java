package org.firstinspires.ftc.teamcode.opsmodes;

/*
    Camera following - but only when the camera is attached
    to a DC Motor
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name="ChinmayOpMode", group="TeleOp")
public class ChinmayOpMode extends LinearOpMode {
    private int speed = 0;

    private double ROT_TOL = 4.0;
    private final int incAmount = 5;
    private boolean wasGoingLeft = false;
    private int curPos = 0;

    private DcMotorEx leftFront = null;
    @Override
    public void runOpMode() throws InterruptedException {
        TestBrain tBrain = new TestBrain(hardwareMap);

        DcMotorEx camSwivel = hardwareMap.get(DcMotorEx.class, "leftFront");
        camSwivel.setTargetPosition(0);
        camSwivel.setPower(0.00);
        camSwivel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.setNumDecimalPlaces(1, 3);

        waitForStart();

        telemetry.speak("please end my suffering");

        while(opModeIsActive()){
            AprilTagPoseFtc tag = tBrain.getCloestTagPose();

            telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size());
            if(tag != null) {
                camSwivel.setPower(1.0);
                telemetry.addData("Range", tag.range);
                telemetry.addData("Bearing", tag.bearing);;

                if(tag.bearing > 0 + ROT_TOL){
                    telemetry.addLine("tag.bearing > 0");
                    wasGoingLeft = true;
                    curPos = camSwivel.getCurrentPosition() - incAmount;
                }else if(tag.bearing < 0 - ROT_TOL){
                    telemetry.addLine("tag.bearing < 0");
                    wasGoingLeft = false;
                    curPos = camSwivel.getCurrentPosition() + incAmount;
                }else{
                    // nothinig..
                }
                camSwivel.setTargetPosition(curPos);

                telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", tag.x, tag.y, tag.z);
            }else{
                /*if(wasGoingLeft){
                    camSwivel.setTargetPosition(curPos + lastDitch);
                }else{
                    camSwivel.setTargetPosition(curPos - lastDitch);
                }*/
                telemetry.addLine("No Pose avaliable");
            }
            telemetry.addData("Target Pos", curPos);
            telemetry.addData("FPS", (int)tBrain.getFPS());
            telemetry.update();
        }

        telemetry.speak("im dying");
    }
}
