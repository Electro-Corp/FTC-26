package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Sarah PTO", group="Linear OpMode")
public class SarahPTO extends LinearOpMode {
    private double STATICPOSITION = 0.0;
    private double FINALPOSITION = 0.5;
    private Servo ptoServo= null;
    private DcMotorEx ptoMotor = null;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ptoServo = hardwareMap.get(Servo.class, "servoT");//PUT IN NAMEEEEE
        ptoMotor = hardwareMap.get(DcMotorEx.class, "rightRear"); //PUT NAME HERE TOOOOOO

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                ptoServo.setPosition(STATICPOSITION);
                timer.reset(); // Resets the timer to zero
                sleep(1000);
                ptoMotor.setPower(.5);
            }
            if(gamepad1.b){
                ptoMotor.setPower(0.0);
                sleep(1000);
                ptoServo.setPosition(FINALPOSITION);
            }
        }
    }
}
