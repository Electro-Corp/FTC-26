package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private double speed;
    public Intake (HardwareMap hardwareMap){
        speed = -1.0;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void setSpeed(double s){
        this.speed = s;
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
