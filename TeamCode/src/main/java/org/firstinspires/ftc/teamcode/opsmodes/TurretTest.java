package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Turret")
public class TurretTest extends LinearOpMode {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor"); //PUT NAMES IN
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                leftMotor.setPower(.6);
                rightMotor.setPower(1);
            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            telemetry.addData("Button A", gamepad1.a);
            telemetry.update();
        }
    }
}
