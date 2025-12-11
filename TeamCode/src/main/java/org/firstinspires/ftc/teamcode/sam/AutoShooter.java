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

    // Constants
    private static final double L_KICKER_WAIT = 0.8;
    private static final double L_KICKER_SHOOT = 0.574;
    private static final double M_KICKER_WAIT = 0.6145;
    private static final double M_KICKER_SHOOT = 0.4175;
    private static final double R_KICKER_WAIT = 0.705;
    private static final double R_KICKER_SHOOT = 0.5305;

    private static final long KICKER_PULSE_MS = 600;
    private static final double SPINNER_SPEED_NEAR = -1360;
    private static final double SPINNER_SPEED_FAR = -7000;

    // Final vars
    private final ColorSensors colorSensors;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;
    private final Servo leftKicker;
    private final Servo midKicker;
    private final Servo rightKicker;
    private final TestBrain testBrain;

    private BallColor[] loadedColors;
    // ballIndex is the index into the *pattern* (0,1,2)
    private int ballIndex = 0;
    private Pattern pattern = Pattern.GPP;

    // firingOrder[i] = which kicker index (0/1/2) to fire for pattern position i
    private final int[] firingOrder = new int[]{0, 1, 2};
    private boolean firingOrderDirty = true;

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
        firingOrderDirty = true;
    }

    public Action readBallColors() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                loadedColors = colorSensors.readAllColors();
                firingOrderDirty = true;
                telemetryPacket.put("loadedL", loadedColors[0].toString());
                telemetryPacket.put("loadedM", loadedColors[1].toString());
                telemetryPacket.put("loadedR", loadedColors[2].toString());
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
     * Compute the mapping from pattern positions to physical kicker indexes.
     * firingOrder[i] = kicker index (0 = left, 1 = mid, 2 = right) that matches pattern color i.
     */
    private void computeFiringOrderIfNeeded(TelemetryPacket p) {
        if (!firingOrderDirty) {
            return;
        }

        firingOrderDirty = false;
        ballIndex = 0; // reset pattern position when we get fresh data

        // No colors yet. Default to identity mapping.
        if (loadedColors == null || loadedColors.length < 3) {
            firingOrder[0] = 0;
            firingOrder[1] = 1;
            firingOrder[2] = 2;
            if (p != null) {
                p.put("firingOrder0", firingOrder[0]);
                p.put("firingOrder1", firingOrder[1]);
                p.put("firingOrder2", firingOrder[2]);
            }
            return;
        }

        BallColor[] desired = pattern.getColors();
        boolean[] used = new boolean[3];

        for (int i = 0; i < 3; i++) {
            BallColor targetColor = desired[i];
            int chosen = -1;

            // First pass: find an unused slot that matches the target color
            for (int j = 0; j < 3; j++) {
                if (!used[j] && loadedColors[j] == targetColor) {
                    chosen = j;
                    break;
                }
            }

            // Fallback: pick any unused slot if no exact color match found
            if (chosen == -1) {
                for (int j = 0; j < 3; j++) {
                    if (!used[j]) {
                        chosen = j;
                        break;
                    }
                }
            }

            // Extra safety fallback
            if (chosen == -1) {
                chosen = 0;
            }

            firingOrder[i] = chosen;
            used[chosen] = true;
        }

        if (p != null) {
            p.put("pattern", pattern.toString());
            p.put("firingOrder0", firingOrder[0]);
            p.put("firingOrder1", firingOrder[1]);
            p.put("firingOrder2", firingOrder[2]);
        }
    }

    /**
     * Based on the current ballIndex (pattern position), opens the appropriate kicker,
     * waits for it to shoot, and retracts it.
     * Uses firingOrder[ballIndex] to choose which kicker to fire.
     */
    public Action fireNextBall() {
        return new FireNextBallAction();
    }

    private class FireNextBallAction implements Action {

        private Servo kicker;
        private double shootPos;
        private double restPos;
        private int patternPos;
        private int kickerIndex;

        private boolean initialized = false;
        private long startTimeMs;

        FireNextBallAction() {
            // patternPos is which pattern color we are shooting now (0,1,2)
            patternPos = ballIndex;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            // Ensure we have a valid mapping before the first call
            if (!initialized) {
                computeFiringOrderIfNeeded(p);

                // sanitize pattern position
                if (patternPos < 0 || patternPos > 2) {
                    patternPos = 0;
                }

                // Map pattern position to actual kicker index
                kickerIndex = firingOrder[patternPos];

                switch (kickerIndex) {
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

                // Prepare the next pattern position for the next ball
                ballIndex = (ballIndex + 1) % 3;

                // First actual motion
                kicker.setPosition(shootPos);
                startTimeMs = System.currentTimeMillis();
                initialized = true;

                p.put("patternPos", patternPos);
                p.put("kickerIndex", kickerIndex);
                p.put("state", "EXTEND");
                return true;
            }

            long elapsed = System.currentTimeMillis() - startTimeMs;

            if (elapsed < KICKER_PULSE_MS) {
                p.put("patternPos", patternPos);
                p.put("kickerIndex", kickerIndex);
                p.put("state", "HOLD");
                p.put("elapsed", elapsed);
                return true;
            }

            // Start retracting
            kicker.setPosition(restPos);

            p.put("patternPos", patternPos);
            p.put("kickerIndex", kickerIndex);
            p.put("state", "RETRACT");
            p.put("elapsed", elapsed);

            return false;
        }
    }

    public Action readObelisk() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                for (int i = 21; i <= 23; i++) {
                    if (testBrain.getTagID(i) != null) {
                        pattern = Pattern.fromNum(i);
                        firingOrderDirty = true;
                    }
                }

                telemetryPacket.put("pattern", pattern.toString());
                return false;
            }
        };
    }
}
