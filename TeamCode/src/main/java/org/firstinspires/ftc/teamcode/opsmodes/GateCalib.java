package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "GateCalib")
public class GateCalib extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo lKick = hardwareMap.get(Servo.class, "lKick");
        Servo mKick = hardwareMap.get(Servo.class,"mKick");
        Servo rKick = hardwareMap.get(Servo.class,"rKick");

        int servoNumber = 0;
        double lPos = 0.6;
        double mPos = 0.5;
        double rPos = 0.5;
        boolean changeDown = false;
        double diff = 0.0005;

        waitForStart();

        while(!isStopRequested()){

            if (gamepad1.dpad_right) {
                if (!changeDown) {
                    servoNumber++;
                    if (servoNumber > 2)
                        servoNumber = 0;
                }
                changeDown = true;
            } else {
                changeDown = false;
            }

            switch (servoNumber) {
                case 0: //LEFT KICKER
                    if (gamepad1.dpad_up) lPos += diff;
                    if (gamepad1.dpad_down) lPos -= diff;
                    lKick.setPosition(lPos);
                    telemetry.addLine("CURRENT IS LEFT");
                    break;
                case 1:
                    if (gamepad1.dpad_up) mPos += diff;
                    if (gamepad1.dpad_down) mPos -= diff;
                    mKick.setPosition(mPos);
                    telemetry.addLine("CURRENT IS MIDDLE");
                    break;
                case 2:
                    if (gamepad1.dpad_up) rPos += diff;
                    if (gamepad1.dpad_down) rPos -= diff;
                    rKick.setPosition(rPos);
                    telemetry.addLine("CURRENT IS RIGHT");
                    break;
            }

            telemetry.addData("LEFT POST", lPos);
            telemetry.addData("MIDDLE POS",mPos);
            telemetry.addData("RIGHT POS", rPos);
            telemetry.update();
        }
    }
}
