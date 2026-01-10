package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter implements Runnable{

    private volatile boolean stop = false;

    @Override
    public void run() {
        while (!stop) {
            update();
            try {
                Thread.sleep(5); // small delay so we don't eat up 100%
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public enum ShooterState {
        STOPPED, WAITING_FOR_SPIN_UP, SPIN_UP_HOLD, SHOOTING, REVERSE, WAIT_FOR_KICKER
    }

    //Constants
    public static final double L_KICKER_WAIT = 0.771;
    public static final double L_KICKER_SHOOT = 0.574;
    public static final double M_KICKER_WAIT = 0.6375;
    public static final double M_KICKER_SHOOT = 0.4175;
    public static final double R_KICKER_WAIT = 0.55;
    public static final double R_KICKER_SHOOT = 0.723;

    private static final float COLOR_GAIN = 30.5f;

    private boolean shouldLShoot = false;
    private boolean shouldRShoot = false;
    private boolean shouldMShoot = false;

    private static final long SPIN_UP_TIME_MS = 1800;
    private static final long SPIN_AFTER_SHOOT_MS = 200;
    private static final long PAUSE_UNTIL_GATE_OPEN = 400;
    private static final long PULL_KICKER_BACK = 750;
    public double SPINNER_SPEED_NEAR = -1300;
    public static final double SPINNER_SPEED_FAR = -7000;

    //Final vars
    private final ColorSensors colorSensors;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo leftKicker;
    private final Servo midKicker;
    private final Servo rightKicker;
    private BallColor[] loadedColors;
    private final boolean readColorsOnce; //AUTO only reads the color once

    private ShooterState state = ShooterState.STOPPED;
    private long stateStartTime = 0;

    private boolean holdSpin = false;

    public Shooter(HardwareMap hardwareMap, ColorSensors colorSensors, boolean readColorsOnce) {
        this.colorSensors = colorSensors;
        this.readColorsOnce = readColorsOnce;
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        this.leftKicker = hardwareMap.get(Servo.class, "lKick");
        this.midKicker = hardwareMap.get(Servo.class, "mKick");
        this.rightKicker = hardwareMap.get(Servo.class,"rKick");
        this.loadedColors = colorSensors.readAllColors();

        this.shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickersWait();
    }

    public double getVelocity(){
        return (Math.abs(shooterLeft.getVelocity()) + Math.abs(shooterRight.getVelocity())) / 2.0;
    }

    public double getLeftVelocity(){
        return Math.abs(shooterLeft.getVelocity());
    }

    public double getRightVelocity(){
        return Math.abs(shooterRight.getVelocity());
    }

    public BallColor[] getLoadedColors() {
        return loadedColors;
    }

    private void resetShootFlags() {
        shouldLShoot = shouldMShoot = shouldRShoot = false;
    }

    public void shootDistance(double distance) {
        if (state != ShooterState.SPIN_UP_HOLD){
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        }
        else {
            setState(ShooterState.SHOOTING);
        }
        shooterLeft.setVelocity(distance * 10);
        shooterRight.setVelocity(distance * -10);
    }

    public void shootFar() {
        if (state != ShooterState.SPIN_UP_HOLD){
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        }
        else {
            setState(ShooterState.SHOOTING);
        }
        shooterLeft.setVelocity(SPINNER_SPEED_FAR);
        shooterRight.setVelocity(-SPINNER_SPEED_FAR);
    }

    public void stopShoot(){
        holdSpin = false;
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
        kickersWait();
        setState(ShooterState.STOPPED);
    }

    public void shootNear() {
        if (state != ShooterState.SPIN_UP_HOLD){
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        }
        else {
            //kickersShoot();
            setState(ShooterState.SHOOTING);
        }
        shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
        shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
    }

    public void spinUp(boolean fast){
        spinUp(fast, true);
    }

    public void spinUp(boolean fast, boolean hold){
        holdSpin = hold;
        setState(ShooterState.SPIN_UP_HOLD);
        if(fast) {
            shooterLeft.setVelocity(SPINNER_SPEED_FAR);
            shooterRight.setVelocity(-SPINNER_SPEED_FAR);
        }else{
            shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
            shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
        }
    }

    public void reverse(boolean fast){
        setState(ShooterState.REVERSE);
        if(fast) {
            shooterLeft.setVelocity(-SPINNER_SPEED_FAR / 10);
            shooterRight.setVelocity(SPINNER_SPEED_FAR / 10);
        }else{
            shooterLeft.setVelocity(-SPINNER_SPEED_NEAR / 10);
            shooterRight.setVelocity(SPINNER_SPEED_NEAR / 10);
        }
    }


    public void update() {
        long elapsed = System.currentTimeMillis() - stateStartTime;
        switch (state) {
            case WAITING_FOR_SPIN_UP:
                //if (/*elapsed >= SPIN_UP_TIME_MS ||*/ (Math.abs(SPINNER_SPEED_NEAR) - 10 < getVelocity() && Math.abs(SPINNER_SPEED_NEAR) + 10 > getVelocity())) {
                    kickersWait();
                    setState(ShooterState.SHOOTING);
                //}
                break;
            case SPIN_UP_HOLD:
                break;
            case WAIT_FOR_KICKER:
                if(elapsed >= PULL_KICKER_BACK){
                    kickersWait();
                    resetWhatToShoot();
                    if(!holdSpin){
                        shooterLeft.setVelocity(0);
                        shooterRight.setVelocity(0);
                        setState(ShooterState.STOPPED);
                    }else{
                        setState(ShooterState.SPIN_UP_HOLD);
                    }
                }
                break;
            case REVERSE:
                break;
            case SHOOTING:
                if (elapsed >= SPIN_AFTER_SHOOT_MS) {
                    kickersShoot();
                    setState(ShooterState.WAIT_FOR_KICKER);
                }
                break;
            default:
                if(elapsed >= PAUSE_UNTIL_GATE_OPEN){
                    kickersWait();
                    setState(ShooterState.STOPPED);
                }
                break;
        }
    }

    public boolean isShooting(){
        return state != ShooterState.STOPPED;
    }

    public void setState(ShooterState newState) {
        state = newState;
        stateStartTime = System.currentTimeMillis();
    }

    private void kickersShoot() {
        leftKickerShoot();
        midKickerShoot();
        rightKickerShoot();
    }

    public void kickersWait() {
        leftKickerWait();
        midKickerWait();
        rightKickerWait();
    }

    private void leftKickerShoot() {
        if(shouldLShoot) {
            leftKicker.setPosition(L_KICKER_SHOOT);
        }
    }

    private void midKickerShoot() {
        if(shouldMShoot) {
            midKicker.setPosition(M_KICKER_SHOOT);
        }
    }

    private void rightKickerShoot() {
        if(shouldRShoot) {
            rightKicker.setPosition(R_KICKER_SHOOT);
        }
    }

    private void leftKickerWait() {
        leftKicker.setPosition(L_KICKER_WAIT);
    }

    private void midKickerWait() {
        midKicker.setPosition(M_KICKER_WAIT);
    }

    private void rightKickerWait() {
        rightKicker.setPosition(R_KICKER_WAIT);
    }

    public void setToShootAll(){
        shouldLShoot = true;
        shouldMShoot = true;
        shouldRShoot = true;
    }

    private void resetWhatToShoot(){
        shouldMShoot = false;
        shouldRShoot = false;
        shouldLShoot = false;
    }

    public void setShootSpecific(boolean a, boolean b, boolean c){
        shouldLShoot = a;
        shouldMShoot = b;
        shouldRShoot = c;
    }

    public boolean shootColorFar(BallColor color){
        return shootColor(color, SPINNER_SPEED_FAR);
    }

    public boolean shootColorNear(BallColor color){
        return shootColor(color, SPINNER_SPEED_NEAR);
    }

    private boolean shootColor(BallColor color, double speed) {
        // Choose live readings or static loaded colors
        BallColor[] colors = readColorsOnce ? loadedColors : new BallColor[]{
                colorSensors.readLeftColor(),
                colorSensors.readMidColor(),
                colorSensors.readRightColor()
        };

        // Reset shooting flags
        resetShootFlags();
        // Find first matching index
        for (int i = 0; i < 3; i++) {
            if (colors[i] == color) {
                if (i == 0) shouldLShoot = true;
                if (i == 1) shouldMShoot = true;
                if (i == 2) shouldRShoot = true;
                break; // important: only shoot one ball
            }
        }

        // Change shooter state based on spin up
        setState(state != ShooterState.SPIN_UP_HOLD
                ? ShooterState.WAITING_FOR_SPIN_UP
                : ShooterState.SHOOTING);

        shooterLeft.setVelocity(speed);
        shooterRight.setVelocity(-speed);

        return shouldLShoot || shouldMShoot || shouldRShoot;
    }

    public void stopShooterThread(){
        this.stop = true;
    }

    public ShooterState getState(){
        return state;
    }

    public void updateLoadedColors(){
        this.loadedColors = colorSensors.readAllColors();
    }

    public void setPID(PIDFCoefficients pid){
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
    }


}