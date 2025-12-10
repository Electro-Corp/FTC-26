package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="two motor low speed test")
public class motorLowSpeed extends LinearOpMode {
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            shooterLeft.setVelocity(gamepad1.right_trigger * 100);
            shooterRight.setVelocity(gamepad1.right_trigger * -100);
        }
    }
}
