package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Sarah Controller OpMode:)")
public class SarahControllerOpMode extends LinearOpMode {
    //variables
    private DcMotorEx leftFront = null;
    private boolean toggle = false;
    private boolean pressed = false;


    @Override
    public void runOpMode() throws InterruptedException {
        //code when init
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        waitForStart();
        //code after init

        while(opModeIsActive()){
            //code after init looping
            /* if (gamepad1.a) {
                leftFront.setPower(.5);
            } else {
                leftFront.setPower(0);
            }
            if(gamepad1.dpad_left){
                leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(gamepad1.dpad_right){
                leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            } */
            if (gamepad1.a && !pressed){
                toggle = !toggle;
                pressed = true;
            } else if (!gamepad1.a) {
                pressed = false;
            }

            telemetry.addData("pressed", pressed);
            telemetry.addData("toggle", toggle);
            telemetry.update();

            float power = gamepad1.left_stick_x;
            if(toggle){
                power /= 2f;
            }
            leftFront.setPower(power);
            if(gamepad1.b){
                gamepad1.rumble(1000);
            } //rumble doesn't work :(


        }

    }
}
