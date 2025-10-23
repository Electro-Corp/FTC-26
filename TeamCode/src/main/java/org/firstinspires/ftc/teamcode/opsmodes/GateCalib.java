package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "GateCalib")
public class GateCalib extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo gate = hardwareMap.get(Servo.class, "gate");

        double pos = 0.5;

        waitForStart();

        while(!isStopRequested()){
            gate.setPosition(pos);

            if(gamepad1.dpad_up){
                pos += 0.0005;
            }else if(gamepad1.dpad_down){
                pos -= 0.0005;
            }

            telemetry.addData("POS", pos);
            telemetry.update();
        }
    }
}
