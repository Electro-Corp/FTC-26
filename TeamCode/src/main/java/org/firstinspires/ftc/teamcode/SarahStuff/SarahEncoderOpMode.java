package org.firstinspires.ftc.teamcode.SarahStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Sarah Encoder :D")
public class SarahEncoderOpMode extends LinearOpMode {
    private DcMotorEx leftDrive = null;
    private int numTics = 0;
    private int newLeftTarget = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        newLeftTarget = 100;

        waitForStart();
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while(opModeIsActive()){
            numTics = leftDrive.getCurrentPosition();
            telemetry.addData("Position at",  "%7d", numTics);
            telemetry.update();

            if(numTics == 100){
                telemetry.addLine("You WINNNN!!!");
            }

            if(gamepad1.a){
                leftDrive.setTargetPosition(numTics + 538);
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setPower(.25);
            }

            if(gamepad1.b){
                leftDrive.setTargetPosition(numTics - 538);
                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftDrive.setPower(.25);
            }

            if(!leftDrive.isBusy()){
                Thread.sleep(2000);
                leftDrive.setPower(0);
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }



        }

    }
}
