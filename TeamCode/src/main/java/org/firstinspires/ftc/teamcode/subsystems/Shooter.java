package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter implements Runnable{

    private boolean stop = false;

    @Override
    public void run() {
        while(!stop) update();
    }

    public enum ShooterState {
        STOPPED, WAITING_FOR_SPIN_UP, SHOOTING
    }

    private static final double GATE_CLOSED = 0.8015;
    private static final double GATE_OPEN = 0.5665;

    private boolean gateOpen = false;

    private static final long SPIN_UP_TIME_MS = 2500;
    private static final long SPIN_AFTER_SHOOT_MS = 1000;
    private static final double SPINNER_SPEED_NEAR = -1450;
    private static final double SPINNER_SPEED_FAR = -1545;

    private final DcMotorEx shooter;
    private final Servo gate;

    private ShooterState state = ShooterState.STOPPED;
    private long stateStartTime = 0;

    public long tickVelocity = 1;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        gate = hardwareMap.get(Servo.class, "gate");

        openGate();
    }

    public double getVelocity(){
        return shooter.getVelocity();
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
        //closeGate();
        setState(ShooterState.STOPPED);
    }

    public void shootThreeFar(){
        for(int i = 0; i < 3; i++) shootFar();
    }

    public void shootNear() {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooter.setVelocity(SPINNER_SPEED_NEAR);
    }

    private long lastPosition = 1;
    private long lastTime = System.currentTimeMillis();

    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;
        switch (state) {
            case WAITING_FOR_SPIN_UP:
                if (elapsed >= SPIN_UP_TIME_MS) {
                    closeGate();
                    setState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                if (elapsed >= SPIN_AFTER_SHOOT_MS) {
                    openGate();
                    shooter.setVelocity(0);
                    setState(ShooterState.STOPPED);
                }
                break;
            default:
                // STOPPED: no action needed
                break;
        }
        tickVelocity = (shooter.getCurrentPosition() - lastPosition / System.currentTimeMillis() - lastTime);
        lastPosition = shooter.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    public boolean isShooting(){
        return state != ShooterState.STOPPED;
    }

    private void setState(ShooterState newState) {
        state = newState;
        stateStartTime = System.currentTimeMillis();
    }

    public void openGate() {
        gate.setPosition(GATE_OPEN);
        gateOpen = true;
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
        gateOpen = false;
    }

    public void toggleGate(){
        if(gateOpen) closeGate();
        else openGate();
    }

    public void stopShooterThread(){
        this.stop = true;
    }

    public ShooterState getState(){
        return state;
    }
}
