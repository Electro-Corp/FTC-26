package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private enum ShooterState {
        STOPPED, WAITING_FOR_SPIN_UP, SHOOTING
    }

    private static final double GATE_CLOSED = 0.2915;
    private static final double GATE_OPEN = 0.2255;

    private static final long SPIN_UP_TIME_MS = 3000;
    private static final long SPIN_AFTER_SHOOT_MS = 1000;
    private static final double SPINNER_SPEED_NEAR = -3000;
    private static final double SPINNER_SPEED_FAR = -50000;

    private final DcMotorEx shooter;
    private final Servo gate;

    private ShooterState state = ShooterState.STOPPED;
    private long stateStartTime = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");

        closeGate();
    }

    public void shootDistance(double distance) {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooter.setVelocity(distance * 10);
    }

    public void shootFar() {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooter.setVelocity(SPINNER_SPEED_FAR);
    }

    public void stopShoot(){
        shooter.setPower(0.0);
        closeGate();
    }

    public void shootThreeFar(){
        for(int i = 0; i < 3; i++) shootFar();
    }

    public void shootNear() {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooter.setVelocity(SPINNER_SPEED_NEAR);
    }

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;
        switch (state) {
            case WAITING_FOR_SPIN_UP:
                if (elapsed >= SPIN_UP_TIME_MS) {
                    openGate();
                    setState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                if (elapsed >= SPIN_AFTER_SHOOT_MS) {
                    closeGate();
                    shooter.setVelocity(0);
                    setState(ShooterState.STOPPED);
                }
                break;
            default:
                // STOPPED: no action needed
                break;
        }
    }

    public boolean isShooting(){
        return state != ShooterState.STOPPED;
    }

    private void setState(ShooterState newState) {
        state = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void openGate() { gate.setPosition(GATE_OPEN); }

    private void closeGate() { gate.setPosition(GATE_CLOSED); }
}
