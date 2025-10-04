package org.firstinspires.ftc.teamcode.SarahStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Sarah OpMode:)", group="Linear OpMode")
public class SarahOpMode extends LinearOpMode{

    private int awesomeness = 1000000;
    private double lameness = 34.982;
    private DcMotorEx leftFront = null;
    private double motorPower = 1;
    private boolean reverse = true;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");


        waitForStart();

        leftFront.setPower(motorPower);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive()){
            if(reverse) {
                motorPower -= .001;
                if (motorPower <= -1) {
                    reverse = false;
                }
            }
            if(!reverse) {
                motorPower += .001;
                if (motorPower >= 1) {
                    reverse = true;
                }
            }
            leftFront.setPower(motorPower);
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }


        /* while(opModeIsActive()) {
            telemetry.addLine("hiiiiiiii");
            telemetry.addData("Sarah awesomeness", awesomeness); // awesomeness: 1000000
            telemetry.addData("Paul lameness", lameness); // lameness: -34.982
            telemetry.update();

        } */
        leftFront.setPower(0);
    }

}
