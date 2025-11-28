package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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

    private static final double L_KICKER_WAIT = 0.8375;
    private static final double L_KICKER_SHOOT = 0.5665;
    private static final double M_KICKER_WAIT = 0.8375;
    private static final double M_KICKER_SHOOT = 0.5665;
    private static final double R_KICKER_WAIT = 1;
    private static final double R_KICKER_SHOOT = 0.5665;

    private boolean leftKickerShooting = false;
    private boolean midKickerShooting = false;
    private boolean rightKickerShooting = false;

    private static final long SPIN_UP_TIME_MS = 2500;
    private static final long SPIN_AFTER_SHOOT_MS = 1000;
    private static final double SPINNER_SPEED_NEAR = -1450;
    private static final double SPINNER_SPEED_FAR = -1545;

    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo leftKicker;
    private final Servo midKicker;
    private final Servo rightKicker;

    // Color Sensors
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor midColor;
    private NormalizedColorSensor rightColor;

    private ShooterState state = ShooterState.STOPPED;
    private long stateStartTime = 0;

    public long tickVelocity = 1;

    public Shooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        leftKicker = hardwareMap.get(Servo.class, "lKick");
        midKicker = hardwareMap.get(Servo.class, "mKick");
        rightKicker = hardwareMap.get(Servo.class,"rKick");

        // Init color sensors
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        midColor = hardwareMap.get(NormalizedColorSensor.class, "midColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        kickersWait();
    }

    public double getVelocity(){
        return shooterLeft.getVelocity();
    }

    public void shootDistance(double distance) {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooterLeft.setVelocity(distance * 10);
        shooterRight.setVelocity(distance * -10);
    }

    public void shootFar() {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooterLeft.setVelocity(SPINNER_SPEED_FAR);
        shooterRight.setVelocity(-SPINNER_SPEED_FAR);
    }

    public void stopShoot(){
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
        //closeGate();
        setState(ShooterState.STOPPED);
    }

    public void shootThreeFar(){
        for(int i = 0; i < 3; i++) shootFar();
    }

    public void shootNear() {
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
        shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
    }


    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;
        switch (state) {
            case WAITING_FOR_SPIN_UP:
                if (elapsed >= SPIN_UP_TIME_MS) {
                    kickersWait();
                    setState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                if (elapsed >= SPIN_AFTER_SHOOT_MS) {
                    kickersShoot();
                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
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

    public void kickersShoot() {
        leftKickerShoot();
        midKickerShoot();
        rightKickerShoot();
    }

    public void kickersWait() {
        leftKickerWait();
        midKickerWait();
        rightKickerWait();
    }

    public void leftKickerShoot() {
        leftKicker.setPosition(L_KICKER_SHOOT);
        leftKickerShooting = true;
    }

    public void midKickerShoot() {
        midKicker.setPosition(M_KICKER_SHOOT);
        midKickerShooting = true;
    }

    public void rightKickerShoot() {
        rightKicker.setPosition(R_KICKER_SHOOT);
        rightKickerShooting = true;
    }

    public void leftKickerWait() {
        leftKicker.setPosition(L_KICKER_WAIT);
        leftKickerShooting = false;
    }

    public void midKickerWait() {
        midKicker.setPosition(M_KICKER_WAIT);
        midKickerShooting = false;
    }

    public void rightKickerWait() {
        rightKicker.setPosition(R_KICKER_WAIT);
        rightKickerShooting = false;
    }

    public void toggleLeftKicker(){
        if(leftKickerShooting) leftKickerWait();
        else leftKickerShoot();
    }

    public void toggleMidKicker() {
        if(midKickerShooting) midKickerWait();
        else midKickerShoot();
    }

    public void toggleRightKicker() {
        if(rightKickerShooting) rightKickerWait();
        else rightKickerShoot();
    }

    public void stopShooterThread(){
        this.stop = true;
    }

    public ShooterState getState(){
        return state;
    }
}
