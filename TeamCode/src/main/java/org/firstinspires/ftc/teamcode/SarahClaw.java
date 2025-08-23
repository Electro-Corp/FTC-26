package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SarahClaw {

    private Servo claw;
    public SarahClaw (HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "ClawServo");
    }

    public void openClaw(){
        claw.setPosition(.2);
    }

    public void closeClaw(){
        claw.setPosition(.6);
    }
}
