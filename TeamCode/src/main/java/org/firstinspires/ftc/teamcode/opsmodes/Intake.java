package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intakeMotor;
    private double speed;
    public Intake (HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void go(){
        intakeMotor.setPower(speed);
    }

    public void reverse(){
        intakeMotor.setPower(-speed);
    }

    public void stop(){
        intakeMotor.setPower(0);
    }

}
