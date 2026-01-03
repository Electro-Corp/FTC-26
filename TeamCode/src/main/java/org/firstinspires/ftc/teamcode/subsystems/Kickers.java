package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kickers {
    public class Config{
        private static final double L_KICKER_WAIT = 0.771;
        private static final double L_KICKER_SHOOT = 0.574;
        private static final double M_KICKER_WAIT = 0.6375;
        private static final double M_KICKER_SHOOT = 0.4175;
        private static final double R_KICKER_WAIT = 0.58;
        private static final double R_KICKER_SHOOT = 0.723;
    }

    public enum Position{
        LEFT, MID, RIGHT
    }

    private final Servo leftKicker;
    private final Servo midKicker;
    private final Servo rightKicker;

    public Kickers(HardwareMap hardwareMap){
        this.leftKicker = hardwareMap.get(Servo.class, "lKick");
        this.midKicker = hardwareMap.get(Servo.class, "mKick");
        this.rightKicker = hardwareMap.get(Servo.class,"rKick");
    }

    public void fireKicker(Position pos){

    }
}
