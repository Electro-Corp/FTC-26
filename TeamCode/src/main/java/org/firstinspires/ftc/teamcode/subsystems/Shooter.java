package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    private static final float COLOR_GAIN = 7.5f;

    private boolean leftKickerShooting = false;
    private boolean midKickerShooting = false;
    private boolean rightKickerShooting = false;

    private boolean shouldLShoot = false;
    private boolean shouldRShoot = false;
    private boolean shouldMShoot = false;

    private static final long SPIN_UP_TIME_MS = 1800;
    private static final long SPIN_AFTER_SHOOT_MS = 1000;
    private static final long PAUSE_UNTIL_GATE_OPEN = 1000;
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

        kickersWait();
    }

    public double getVelocity(){
        return (shooterLeft.getVelocity() + Math.abs(shooterRight.getVelocity())) / 2;
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
        setState(ShooterState.STOPPED);
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
        if(shouldLShoot) {
            leftKicker.setPosition(L_KICKER_SHOOT);
            leftKickerShooting = true;
        }
    }

    public void midKickerShoot() {
        if(shouldMShoot) {
            midKicker.setPosition(M_KICKER_SHOOT);
            midKickerShooting = true;
        }
    }

    public void rightKickerShoot() {
        if(shouldRShoot) {
            rightKicker.setPosition(R_KICKER_SHOOT);
            rightKickerShooting = true;
        }
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

    public void setToShootAll(){
        shouldLShoot = true;
        shouldMShoot = true;
        shouldRShoot = true;
    }

    public void resetWhatToShoot(){
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

    public boolean shootColor(BallColor color, double speed){
        // if one replaced the "else if" with "if"'s
        // multiple balls of the same color could be fired
        if(whatColor(getLeftColor()) == color)
            shouldLShoot = true;
        else if(whatColor(getRightColor()) == color)
            shouldRShoot = true;
        else if(whatColor(getMidColor()) == color)
            shouldMShoot = true;
        setState(ShooterState.WAITING_FOR_SPIN_UP);
        shooterLeft.setVelocity(speed);
        shooterRight.setVelocity(-speed);
        return shouldLShoot || shouldRShoot || shouldMShoot;
    }

    public void stopShooterThread(){
        this.stop = true;
    }

    public ShooterState getState(){
        return state;
    }


    // Color sensor reading
    public static BallColor whatColor(NormalizedRGBA color){
        if(color.green > 0.060){
            if(color.blue > 0.060 && color.blue > color.green){
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