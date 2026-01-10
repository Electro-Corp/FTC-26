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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class AutoShooter {

    // Constants
    private static final long KICKER_PULSE_MS = 600;
    private static final long SHOOTER_RECOVERY_TIMEOUT_MS = 1000;
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

    private double lastTargetVelocity = SPINNER_SPEED_NEAR;
    private BallColor[] loadedColors;
    // ballIndex is the index into the pattern (0, 1, 2)
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
        leftKicker.setPosition(Shooter.L_KICKER_WAIT);
        midKicker.setPosition(Shooter.M_KICKER_WAIT);
        rightKicker.setPosition(Shooter.R_KICKER_WAIT);
    }

    public Action readBallColors() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                loadedColors = colorSensors.readAllColors();
                firingOrderDirty = true;
                ballIndex = 0;

                if (loadedColors != null && loadedColors.length >= 3) {
                    // Compact string like "GPP"
                    String ballsSummary = shortName(loadedColors[0])  + shortName(loadedColors[1]) + shortName(loadedColors[2]);
                    telemetryPacket.put("balls", ballsSummary);
                }

                telemetryPacket.put("pattern", pattern.toString());
                return false;
            }
        };
    }

    // Helper to show a single letter for a BallColor
    private String shortName(BallColor c) {
        if (c == null) return "?";
        switch (c) {
            case GREEN:  return "G";
            case PURPLE: return "P";
            default:     return "?"; // unknown
        }
    }

    public Action spinUp(boolean fast){
        double targetVelocity = fast ? SPINNER_SPEED_FAR : SPINNER_SPEED_NEAR;
        lastTargetVelocity = targetVelocity;
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

            double shooterVelocity = (Math.abs(shooterLeft.getVelocity()) + Math.abs(shooterRight.getVelocity())) / 2.0;

            // keep spinning until we reach the target magnitude
            //return shooterVelocity < Math.abs(targetVelocity);
            return false;
        }
    }

    /**
     * Compute the mapping from pattern positions to physical kicker indexes.
     * firingOrder[i] = kicker index (0 = left, 1 = mid, 2 = right) that matches pattern color i.
     */
    private void computeFiringOrderIfNeeded() {
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
    }

    /**
     * Fires 3 balls in order based on the current pattern and loaded colors.
     */
    public Action fire3balls() {
        return new Fire3BallsAction();
    }

    private class Fire3BallsAction implements Action {

        private Servo kicker;
        private double shootPos;
        private double restPos;
        private int kickerIndex;

        private boolean initialized = false;
        private boolean ballInitialized = false;
        private int ballCount = 0;
        private long startTimeMs;
        private int stage = 0; // 0 = pulse, 1 = recovery

        Fire3BallsAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                computeFiringOrderIfNeeded();
                initialized = true;
            }

            if (!ballInitialized) {
                // Capture current pattern position at run-time
                int patternPos = ballIndex;

                // sanitize pattern position
                if (patternPos < 0 || patternPos > 2) {
                    patternPos = 0;
                }

                // Map pattern position to actual kicker index
                kickerIndex = firingOrder[patternPos];

                switch (kickerIndex) {
                    case 0:
                        kicker = leftKicker;
                        shootPos = Shooter.L_KICKER_SHOOT;
                        restPos = Shooter.L_KICKER_WAIT;
                        break;
                    case 1:
                        kicker = midKicker;
                        shootPos = Shooter.M_KICKER_SHOOT;
                        restPos = Shooter.M_KICKER_WAIT;
                        break;
                    case 2:
                    default:
                        kicker = rightKicker;
                        shootPos = Shooter.R_KICKER_SHOOT;
                        restPos = Shooter.R_KICKER_WAIT;
                        break;
                }

                // Prepare the next pattern position for the next ball
                ballIndex = (ballIndex + 1) % 3;

                kicker.setPosition(shootPos);
                startTimeMs = System.currentTimeMillis();
                stage = 0;
                ballInitialized = true;

                p.put("kicker", "EXTEND " + kickerIndex);
            }

            long elapsed = System.currentTimeMillis() - startTimeMs;

            if (stage == 0) {
                if (elapsed < KICKER_PULSE_MS) {
                    p.put("kicker", "HOLD " + kickerIndex);
                    return true;
                }

                // Start retracting
                kicker.setPosition(restPos);
                stage = 1;
                p.put("kicker", "RETRACT " + kickerIndex);
            }

            double shooterVelocity = (Math.abs(shooterLeft.getVelocity()) + Math.abs(shooterRight.getVelocity())) / 2.0;
            double targetVelocity = Math.abs(lastTargetVelocity);
            boolean atSpeed = shooterVelocity >= targetVelocity;
            boolean timedOut = elapsed >= SHOOTER_RECOVERY_TIMEOUT_MS;

            p.put("shooterVelocity", shooterVelocity);

            if (atSpeed || timedOut) {
                ballCount++;
                ballInitialized = false;
                if (ballCount >= 3) {
                    return false;
                }
            }

            return true;
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
                        ballIndex = 0;
                    }
                }

                telemetryPacket.put("pattern", pattern.toString());
                return false;
            }
        };
    }

    public Action stop(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                return false;
            }
        };
    }
}
