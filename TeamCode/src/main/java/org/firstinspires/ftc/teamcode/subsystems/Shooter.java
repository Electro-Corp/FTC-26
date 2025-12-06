package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter implements Runnable{

    private volatile boolean stop = false;

    @Override
    public void run() {
        while(!stop) update();
    }

    public enum ShooterState {
        STOPPED, WAITING_FOR_SPIN_UP, SPIN_UP_HOLD, SHOOTING
    }

    public enum BallColor{
        PURPLE,
        GREEN,
        UNKNOWN;

        public String toString(){
            switch(this){
                case PURPLE:
                    return "PURPLE";
                case GREEN:
                    return "GREEN";
                default:
                    return "NONE";
            }
        }
    }

    private static final double L_KICKER_WAIT = 0.8;
    private static final double L_KICKER_SHOOT = 0.574;
    private static final double M_KICKER_WAIT = 0.6145;
    private static final double M_KICKER_SHOOT = 0.4175;
    private static final double R_KICKER_WAIT = 0.705;
    private static final double R_KICKER_SHOOT = 0.5305;

    private static final float COLOR_GAIN = 30.5f;

    private boolean shouldLShoot = false;
    private boolean shouldRShoot = false;
    private boolean shouldMShoot = false;

    private static final long SPIN_UP_TIME_MS = 1800;
    private static final long SPIN_AFTER_SHOOT_MS = 1000;
    private static final long PAUSE_UNTIL_GATE_OPEN = 1000;
    private static final double SPINNER_SPEED_NEAR = -1360;
    private static final double SPINNER_SPEED_FAR = -7000;

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

    public boolean readColorsOnce = false;

    public int lastFired = -1, secLastFir = -2;

    public BallColor loadedColors[] = new BallColor[3];

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
        leftColor.setGain(COLOR_GAIN);
        midColor.setGain(COLOR_GAIN);
        rightColor.setGain(COLOR_GAIN);

        readColors();
        kickersWait();
    }

    public double getVelocity(){
        return (shooterLeft.getVelocity() + Math.abs(shooterRight.getVelocity())) / 2;
    }

    public void shootDistance(double distance) {
        if(state != ShooterState.SPIN_UP_HOLD)
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        else
            setState(ShooterState.SHOOTING);
        shooterLeft.setVelocity(distance * 10);
        shooterRight.setVelocity(distance * -10);
    }

    public void shootFar() {
        if(state != ShooterState.SPIN_UP_HOLD)
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        else
            setState(ShooterState.SHOOTING);
        shooterLeft.setVelocity(SPINNER_SPEED_FAR);
        shooterRight.setVelocity(-SPINNER_SPEED_FAR);
    }

    public void stopShoot(){
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
        setState(ShooterState.STOPPED);
    }

    public void shootNear() {
        if(state != ShooterState.SPIN_UP_HOLD)
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        else
            setState(ShooterState.SHOOTING);
        shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
        shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
    }

    public void spinUp(boolean fast){
        setState(ShooterState.SPIN_UP_HOLD);
        if(fast) {
            shooterLeft.setVelocity(SPINNER_SPEED_FAR);
            shooterRight.setVelocity(-SPINNER_SPEED_FAR);
        }else{
            shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
            shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
        }
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
            case SPIN_UP_HOLD:
                break;
            case SHOOTING:
                if (elapsed >= SPIN_AFTER_SHOOT_MS) {
                    kickersShoot();
                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
                    setState(ShooterState.STOPPED);
                    resetWhatToShoot();
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

    private void kickersWait() {
        leftKickerWait();
        midKickerWait();
        rightKickerWait();
    }

    private void leftKickerShoot() {
        if(shouldLShoot) {
            lastFired = 0;
            leftKicker.setPosition(L_KICKER_SHOOT);
        }
    }

    private void midKickerShoot() {
        if(shouldMShoot) {
            lastFired = 1;
            midKicker.setPosition(M_KICKER_SHOOT);
        }
    }

    private void rightKickerShoot() {
        if(shouldRShoot) {
            lastFired = 2;
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

    private boolean shootColor(BallColor color, double speed){
        // if one replaced the "else if" with "if"'s
        // multiple balls of the same color could be fired
        int tmp = lastFired;
        if(!readColorsOnce) {
            if (whatColor(getLeftColor()) == color) {
                shouldLShoot = true;
                lastFired = 0;
            }
            else if (whatColor(getRightColor()) == color) {
                shouldRShoot = true;
                lastFired = 2;
            }
            else if (whatColor(getMidColor()) == color) {
                shouldMShoot = true;
                lastFired = 1;
            }
        }else{
            if (loadedColors[0] == color) {
                shouldLShoot = true;
                lastFired = 0;
            }
            else if (loadedColors[2] == color) {
                shouldRShoot = true;
                lastFired = 2;
            }
            else if (loadedColors[1] == color) {
                shouldMShoot = true;
                lastFired = 1;
            }
        }
        if(state != ShooterState.SPIN_UP_HOLD)
            setState(ShooterState.WAITING_FOR_SPIN_UP);
        else
            setState(ShooterState.SHOOTING);
        shooterLeft.setVelocity(speed);
        shooterRight.setVelocity(-speed);

        if(shouldLShoot || shouldRShoot || shouldMShoot){
            secLastFir = tmp;
        }

        return shouldLShoot || shouldRShoot || shouldMShoot;
    }

    public void readColors(){
        loadedColors[0] = whatColor(getLeftColor());
        loadedColors[1] = whatColor(getMidColor());
        loadedColors[2] = whatColor(getRightColor());
    }

    public void stopShooterThread(){
        this.stop = true;
    }

    public ShooterState getState(){
        return state;
    }


    // Color sensor reading
    public static BallColor whatColor(NormalizedRGBA color){
        if(color.green > 0.100){
            if(color.blue > 0.100 && color.blue > color.green){
                return BallColor.PURPLE;
            }
            return BallColor.GREEN;
        }else{
            return BallColor.UNKNOWN;
        }
    }

    public NormalizedRGBA getLeftColor(){
        return leftColor.getNormalizedColors();
    }

    public NormalizedRGBA getMidColor(){
        return midColor.getNormalizedColors();
    }

    public NormalizedRGBA getRightColor(){
        return rightColor.getNormalizedColors();
    }
}