package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class Limelight {
    Limelight3A limelight;

    private boolean isTeleOp;
    private boolean isRed;
    private int id = 0;
    private Pose3D botPose;
    private LLResult llResult;

    public enum PipelineSwitcher {
        BLUE(0), RED(1), OBELISK(2);

        private final int index;

        private PipelineSwitcher(int index) {
            this.index = index;
        }

        public int index() {
            return index;
        }
    }

    private PipelineSwitcher pipeline = PipelineSwitcher.OBELISK;

    public Limelight (boolean isTeleOp, boolean isRed) {
        this.isTeleOp = isTeleOp;
        this.isRed = isRed;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    /**
     * Direct-construction overload used by OpModes that already have their own hardwareMap
     * and want to pick a starting pipeline explicitly (e.g. the calibration TeleOp). The
     * isTeleOp/isRed auto-switching logic in {@link #update()} is bypassed when this
     * constructor is used because both flags default to false.
     */
    public Limelight(HardwareMap hardwareMap, PipelineSwitcher initialPipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        switchPipeline(initialPipeline);
        limelight.start();
    }

    public void switchPipeline(PipelineSwitcher newPipeline) {
        limelight.pipelineSwitch(newPipeline.index());
        pipeline = newPipeline;
    }

    public PipelineSwitcher getPipeline() {
        return pipeline;
    }

    public void update() {
        llResult = limelight.getLatestResult();

        switch (pipeline) {
            case BLUE:
                break;
            case RED:
                break;
            case OBELISK:
                if (isTeleOp) {
                    if (isRed) switchPipeline(PipelineSwitcher.RED);
                    else switchPipeline(PipelineSwitcher.BLUE);
                } else {

                }

                break;
        }
    }
    public int getID() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = (List<LLResultTypes.FiducialResult>) result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
        }
        return id;
    }

    /**
     * Horizontal angle from the camera crosshair to the target, in degrees.
     * Positive = target is to the right of the crosshair. NaN if no target.
     * Reads the most recent frame directly so callers don't need to call update() first.
     */
    public double getTx() {
        LLResult r = getLatestResult();
        return (r != null && r.isValid()) ? r.getTx() : Double.NaN;
    }

    /**
     * Vertical angle from the camera crosshair to the target, in degrees.
     * Positive = target is above the crosshair. NaN if no target.
     */
    public double getTy() {
        LLResult r = getLatestResult();
        return (r != null && r.isValid()) ? r.getTy() : Double.NaN;
    }

    /**
     * Target area as a percentage of the image (0..100). Useful as a coarse proximity
     * heuristic — area grows roughly with 1/distance^2. NaN if no target.
     */
    public double getTa() {
        LLResult r = getLatestResult();
        return (r != null && r.isValid()) ? r.getTa() : Double.NaN;
    }

    public boolean angleAlign() {
        llResult.getTx();
        return false;
    }

    // ---------------------------------------------------------------------
    // Calibration / auto-aim additions
    // ---------------------------------------------------------------------

    /**
     * Returns the most recent raw Limelight result, refreshing the cached value first.
     * Other helpers below operate on this fresh read instead of the cached llResult,
     * which only updates when {@link #update()} is called.
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /** True if the Limelight currently has a valid target on the active pipeline. */
    public boolean hasTarget() {
        LLResult r = getLatestResult();
        return r != null && r.isValid();
    }

    /**
     * Horizontal floor distance (inches) from the robot origin to the first fiducial
     * seen, computed from the Limelight's 3D target pose in robot space.
     *
     * Why the 3D pose instead of ty + camera-mount-angle trig?
     *   The trig formula is only accurate when the robot is square to the tag (tx ≈ 0).
     *   In the far shooting zone the robot physically cannot face the AprilTag head-on,
     *   so we project the tag's full 3D position onto the floor with hypot(x, z).
     *   This stays correct at any viewing angle.
     *
     * Robot-space convention: x = lateral (right+), z = forward, y = vertical.
     *
     * @return distance in inches, or NaN if no valid target / no pose available.
     */
    public double getDistance() {
        return distanceFrom(getLatestResult());
    }

    /**
     * Internal helper: extract a floor distance from a single LLResult, or NaN if
     * any required field is missing or invalid.
     */
    private static double distanceFrom(LLResult r) {
        if (r == null || !r.isValid()) return Double.NaN;
        java.util.List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
        if (fids == null || fids.isEmpty()) return Double.NaN;

        Pose3D tagPose = fids.get(0).getTargetPoseRobotSpace();
        if (tagPose == null) return Double.NaN;

        Position pos = tagPose.getPosition().toUnit(DistanceUnit.INCH);
        // Floor distance ignores vertical offset (y); we only care about the
        // horizontal range the projectile must travel.
        return Math.hypot(pos.x, pos.z);
    }

    /**
     * Capture multiple successive valid frames and return the mean of distance, tx,
     * ty, and ta. This smooths out noise in the Limelight's 3D pose estimator so
     * each logged calibration sample is a stable reading.
     *
     * The loop deduplicates frames by reference so a tight calling loop can't
     * double-count the same frame between Limelight publishes.
     *
     * @param frames    Maximum number of valid frames to average over.
     * @param timeoutMs Hard cap on total wait time even if {@code frames} not reached.
     * @return aggregated Sample, or null if no valid frame was seen at all.
     */
    public Sample sampleAveraged(int frames, long timeoutMs) {
        long start = System.currentTimeMillis();
        int n = 0;
        double sumDist = 0, sumTx = 0, sumTy = 0, sumTa = 0;
        LLResult last = null;

        while (n < frames && System.currentTimeMillis() - start < timeoutMs) {
            LLResult r = getLatestResult();
            if (r != null && r.isValid() && r != last) {
                double d = distanceFrom(r);
                if (!Double.isNaN(d)) {
                    sumDist += d;
                    sumTx += r.getTx();
                    sumTy += r.getTy();
                    sumTa += r.getTa();
                    n++;
                }
                last = r;
            }
            // Match the 100 Hz Limelight poll rate so we don't spin faster than new
            // frames arrive.
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        if (n == 0) return null;
        return new Sample(sumDist / n, sumTx / n, sumTy / n, sumTa / n, n);
    }

    /** Immutable bundle of an averaged Limelight reading. */
    public static class Sample {
        public final double distance;
        public final double tx, ty, ta;
        public final int frames;

        public Sample(double distance, double tx, double ty, double ta, int frames) {
            this.distance = distance;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.frames = frames;
        }
    }

    /**
     * Suggested yaw command in [-1, 1] that drives tx toward zero. Add this directly
     * to the yaw component of a mecanum drive each loop iteration to auto-aim.
     *
     * The 30.0 divisor normalises tx (Limelight 3A horizontal FOV is ~±27°) into
     * roughly [-1, 1] before scaling by kP. Returns 0 when no target is visible so
     * the robot doesn't spin blindly searching.
     *
     * @param kP proportional gain. Start around 0.5 and tune.
     */
    public double getYawCorrection(double kP) {
        double tx = getTx();
        if (Double.isNaN(tx)) return 0;

        double power = kP * tx / 30.0;
        if (power > 1)  power = 1;
        if (power < -1) power = -1;
        return power;
    }

    /** Escape hatch for code that needs raw Limelight3A access. */
    public Limelight3A raw() {
        return limelight;
    }
}
