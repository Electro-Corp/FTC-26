package org.firstinspires.ftc.teamcode.sam;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.opsmodes.auto.Pattern;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;

public class AutoShooter {


    //Constants
    private static final double L_KICKER_WAIT = 0.8;
    private static final double L_KICKER_SHOOT = 0.574;
    private static final double M_KICKER_WAIT = 0.6145;
    private static final double M_KICKER_SHOOT = 0.4175;
    private static final double R_KICKER_WAIT = 0.705;
    private static final double R_KICKER_SHOOT = 0.5305;

    private static final long KICKER_PULSE_MS = 600;
    private static final double SPINNER_SPEED_NEAR = -1360;
    private static final double SPINNER_SPEED_FAR = -7000;

    //Final vars
    private final ColorSensors colorSensors;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo leftKicker;
    private final Servo midKicker;
    private final Servo rightKicker;
    private final TestBrain testBrain;
    private BallColor[] loadedColors;
    private int ballIndex = 0; // the ball index in the pattern
    private Pattern pattern = Pattern.GPP;


    public AutoShooter(HardwareMap hardwareMap, ColorSensors colorSensors, TestBrain testBrain) {
        this.colorSensors = colorSensors;
        this.testBrain = testBrain;
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        this.leftKicker = hardwareMap.get(Servo.class, "lKick");
        this.midKicker = hardwareMap.get(Servo.class, "mKick");
        this.rightKicker = hardwareMap.get(Servo.class,"rKick");
    }

    public void setPattern(Pattern pattern) {
        this.pattern = pattern;
    }

    public Action readBallColors() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                loadedColors = colorSensors.readAllColors();
                return false;
            }
        };
    }

    public Action spinUp(boolean fast){
        double targetVelocity = fast ? SPINNER_SPEED_FAR : SPINNER_SPEED_NEAR;
        return new SpinUpAction(targetVelocity);
    }

    private class SpinUpAction implements Action {
        private boolean initialized = false;
        private final double targetVelocity;

        public SpinUpAction(double targetVelocity) {
            this.targetVelocity = targetVelocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                shooterLeft.setVelocity(targetVelocity);
                shooterRight.setVelocity(-targetVelocity);
                initialized = true;
            }

            double shooterVelocity = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2;
            telemetryPacket.put("shooterVelocity", shooterVelocity);
            return shooterVelocity < targetVelocity;
        }
    }

    /**
     * Based on the current ballIndex, opens the appropriate kicker, waits for it to shoot, and retracts it.
     * Note that this blocks until the kicker begins retracting
     */
    public Action fireNextBall() {
        return new FireNextBallAction();
    }

    private class FireNextBallAction implements Action {

        private Servo kicker;
        private double shootPos;
        private double restPos;
        private int index;

        private boolean initialized = false;
        private long startTimeMs;

        FireNextBallAction() {
            // sanitize invalid index
            if (ballIndex < 0 || ballIndex > 2) {
                ballIndex = 0;
            }

            this.index = ballIndex;

            switch (index) {
                case 0:
                    kicker = leftKicker;
                    shootPos = L_KICKER_SHOOT;
                    restPos = L_KICKER_WAIT;
                    break;

                case 1:
                    kicker = midKicker;
                    shootPos = M_KICKER_SHOOT;
                    restPos = M_KICKER_WAIT;
                    break;

                case 2:
                default:
                    kicker = rightKicker;
                    shootPos = R_KICKER_SHOOT;
                    restPos = R_KICKER_WAIT;
                    break;
            }

            // Prepare the next ball for the next caller
            ballIndex = (ballIndex + 1) % 3;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                kicker.setPosition(shootPos);
                startTimeMs = System.currentTimeMillis();
                initialized = true;

                p.put("kickerIndex", index);
                p.put("state", "EXTEND");
                return true;
            }

            long elapsed = System.currentTimeMillis() - startTimeMs;

            if (elapsed < KICKER_PULSE_MS) {
                p.put("kickerIndex", index);
                p.put("state", "HOLD");
                return true;
            }

            // Start retracting
            kicker.setPosition(restPos);

            p.put("kickerIndex", index);
            p.put("state", "RETRACT");

            return false;
        }
    }

    public Action readObelisk() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for(int i = 21; i <= 23; i++){
                    if(testBrain.getTagID(i) != null){
                        pattern = Pattern.fromNum(i);
                    }
                }

                return false;
            }
        };
    }
}