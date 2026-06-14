package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static final double L_KICKER_WAIT = 0.8025;
    public static final double L_KICKER_SHOOT = 0.574;
    public static final double M_KICKER_WAIT = 0.65;
    public static final double M_KICKER_SHOOT = 0.4175;
    public static final double R_KICKER_WAIT = 0.543;
    public static final double R_KICKER_SHOOT = 0.723;
    public static final double L_DAM_UP_POS = 0.101;
    public static final double L_DAM_DOWN_POS = 0.5125;

    private static final float COLOR_GAIN = 30.5f;

    private boolean shouldLShoot = false;
    private boolean shouldRShoot = false;
    private boolean shouldMShoot = false;
    private boolean isDamUp = false;

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
    private final Servo leftDam;
    private BallColor[] loadedColors;
    public final boolean readColorsOnce; //AUTO only reads the color once

    private ShooterState state = ShooterState.STOPPED;
    private long stateStartTime = 0;

    private boolean holdSpin = false;

    /**
     * Tracks whether we are inside an active firing session. Set true when a
     * shoot command transitions us out of SPIN_UP_HOLD; cleared by stopShoot().
     * While true, the dam stays DOWN even during the brief SPIN_UP_HOLD windows
     * between successive shots, so we don't waste time raising and lowering it
     * for every ball in a 3-shot sequence.
     */
    private boolean firingSessionActive = false;

    public Shooter(HardwareMap hardwareMap, ColorSensors colorSensors, boolean readColorsOnce) {
        this.colorSensors = colorSensors;
        this.readColorsOnce = readColorsOnce;
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        this.leftKicker = hardwareMap.get(Servo.class, "lKick");
        this.midKicker = hardwareMap.get(Servo.class, "mKick");
        this.rightKicker = hardwareMap.get(Servo.class,"rKick");
        this.leftDam = hardwareMap.get(Servo.class, "leftDam");
        //this.rightDam = hardwareMap.get(Servo.class, "rightDam");

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
        firingSessionActive = true;
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
        firingSessionActive = true;
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
        firingSessionActive = false;
        shooterLeft.setPower(0.0);
        shooterRight.setPower(0.0);
        kickersWait();
        setState(ShooterState.STOPPED);
    }

    public void shootNear() {
        firingSessionActive = true;
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

    /**
     * Re-apply SPINNER_SPEED_NEAR to the flywheel motors. Call this every loop iteration
     * when the upstream code is mutating SPINNER_SPEED_NEAR continuously (e.g. from a
     * Limelight distance lookup) — the original spinUp/shootNear/shootFar methods only
     * latch the velocity once at state-transition time, so without this method the
     * motors keep spinning at whatever speed was first commanded.
     *
     * No-op unless the shooter is in a state where the flywheels should be spinning
     * forward — STOPPED / REVERSE / WAIT_FOR_KICKER are intentionally skipped so we
     * don't fight other commands.
     */
    public void retargetVelocity() {
        switch (state) {
            case SPIN_UP_HOLD:
            case WAITING_FOR_SPIN_UP:
            case SHOOTING:
                shooterLeft.setVelocity(SPINNER_SPEED_NEAR);
                shooterRight.setVelocity(-SPINNER_SPEED_NEAR);
                break;
            default:
                // STOPPED / REVERSE / REVERSE_HUMAN_PLAYER — leave velocity alone.
                break;
        }
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
            shooterLeft.setVelocity(-SPINNER_SPEED_FAR / 8);
            shooterRight.setVelocity(SPINNER_SPEED_FAR / 8);
        }else{
            shooterLeft.setVelocity(-SPINNER_SPEED_NEAR / 8);
            shooterRight.setVelocity(SPINNER_SPEED_NEAR / 8);
        }
    }

    public void update() {
        // TATE PID
//        double p = PIDControl(SPINNER_SPEED_NEAR, shooterLeft.getVelocity());
//        shooterLeft.setPower(p);
//        p = PIDControl(SPINNER_SPEED_NEAR, shooterRight.getVelocity());
//        shooterRight.setPower(-p);

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

        // Dam policy:
        //   Up   while idle / reversing / intaking / pre-spin-up holds BEFORE the
        //        first shot of a session. Keeps balls from dribbling into the
        //        flywheel during intake.
        //   Down once a firing session has started — stays down through the brief
        //        SPIN_UP_HOLD windows between successive shots so we don't waste
        //        time raising and lowering the dam between every ball in a 3-shot
        //        sequence. Cleared by stopShoot().
        //
        // History: previously SPIN_UP_HOLD only raised the dam when readColorsOnce
        // was false (TeleOp), which meant in auto the dam stayed DOWN the whole
        // match — balls fed prematurely during intake. Earlier fix raised it in
        // SPIN_UP_HOLD unconditionally, which then bounced it up between shots.
        // The firingSessionActive flag is the third revision: dam-up by default,
        // but pinned down for the duration of an active firing session.
        if (state == ShooterState.STOPPED
                || state == ShooterState.REVERSE
                || state == ShooterState.REVERSE_HUMAN_PLAYER
                || state == ShooterState.WAITING_FOR_SPIN_UP
                || (state == ShooterState.SPIN_UP_HOLD && !firingSessionActive)) {
            setDamUp();
        } else {
            setDamDown();
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

    public void toggleDam() {
        if (isDamUp) {
            setDamDown();
            isDamUp = false;
        } else {
            setDamUp();
            isDamUp = true;
        }
    }

    public void setDamUp() {
        leftDam.setPosition(L_DAM_UP_POS);
        isDamUp = true;
    }

    public void setDamDown() {
        leftDam.setPosition(L_DAM_DOWN_POS);
        isDamUp = false;
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
        firingSessionActive = true;
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

    /*
        Tate Hunter's PID
     */

    double integralSum = 0;
    public static double Kp = -0.03;
    public static double Ki = 0;
    public static double Kd = 0;
    public static int target = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lasterror = 0;

    public double PIDControl(double reference, double state){
        double error = reference-state;
        integralSum += error * timer.seconds();
        double derivitive = (error - lasterror) / timer.seconds();
        lasterror = error;
        timer.reset();
        double output = (error * Kp) + (derivitive * Kd) + (integralSum * Ki);
        return output;
    }

}