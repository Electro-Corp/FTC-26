package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Sarah IterativeOpMode:)")
public class SarahIterativeOpMode extends OpMode
{
    private DcMotorEx leftFront = null;
    private double power = -1;
    private double change = 0.005;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        telemetry.addLine("POTATOESSSS!");
        telemetry.update();
    }

    @Override
    public void loop() {
        leftFront.setPower(power);
        power += change;
        if (power >= 1 || power <=-1) {
            change = -change;
        }
        telemetry.addData("Power", power);
        telemetry.update();

    }
}
