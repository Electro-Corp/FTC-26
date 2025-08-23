package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Amazing ServoOpMode:)")
public class SarahServoOpMode extends LinearOpMode {
    private double position = 0.5;
    private Servo bestServo = null;


    @Override
    public void runOpMode() throws InterruptedException {
        bestServo = hardwareMap.get(Servo.class, "bestServo");

        waitForStart();
        bestServo.setPosition(position);

        while(opModeIsActive()){

            telemetry.addData("Position", position);
            telemetry.update();

        }
    }
}
