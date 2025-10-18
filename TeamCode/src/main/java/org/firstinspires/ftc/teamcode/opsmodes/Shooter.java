package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private enum ShooterState {
        STOPPED,
        WAITING_FOR_SPIN_UP,
        SHOOTING
    }

    private long SPIN_UP_TIME_MS = 1000;
    private long SPIN_AFTER_SHOOT_MS = 1000;
    private DcMotorEx shooter;
    private Servo gate;
    private ShooterState shooterState = ShooterState.STOPPED;
    private long timestamp = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void shootDistance(double distance){
        shooterState = ShooterState.WAITING_FOR_SPIN_UP;
        timestamp = System.currentTimeMillis();
        shooter.setVelocity(distance*10);
    }

    private void openGate() { //AND SEIZE THE DAY
        gate.setPosition(.5);
    }

    private void closeGate() {
        gate.setPosition(0);
    }

    // This needs to be called continuously in the main ops mode
    public void update() {
        if (shooterState == ShooterState.WAITING_FOR_SPIN_UP) {
            if (System.currentTimeMillis() - timestamp > SPIN_UP_TIME_MS) {
                shooterState = ShooterState.SHOOTING;
                timestamp = System.currentTimeMillis();
                openGate();
            }
        }

        if (shooterState == ShooterState.SHOOTING) {
            if (System.currentTimeMillis() - timestamp > SPIN_AFTER_SHOOT_MS) {
                closeGate();
                shooter.setVelocity(0);
                shooterState = ShooterState.STOPPED;
            }
        }
    }
}
