package org.firstinspires.ftc.teamcode.opsmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name="WheelCount", group="TeleOp")

public class WheelCounter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(STOP_AND_RESET_ENCODER);

        waitForStart();

        while(!isStopRequested()){
            telemetry.addLine("hello");
            telemetry.addData("Pos", shooter.getCurrentPosition());
            telemetry.update();
        }
    }
}
