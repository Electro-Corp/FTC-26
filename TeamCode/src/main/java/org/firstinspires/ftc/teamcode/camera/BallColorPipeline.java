package org.firstinspires.ftc.teamcode.camera;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Downward-facing camera pipeline that classifies each of the 3 ball slots as
 * GREEN, PURPLE, or UNKNOWN using HSV color thresholding.
 *
 * Hardware map name for the camera: "ballCam"
 *
 * The frame is split into 3 equal horizontal strips:
 *   slot 0 = left, slot 1 = centre, slot 2 = right
 * matching the shooter kicker layout (left, middle, right).
 *
 * Tuning: adjust GREEN_* / PURPLE_* constants if detection is unreliable.
 * A calibration opmode (like ColorCalib) that samples HSV values from each
 * ball under competition lighting is the fastest way to dial these in.
 * OpenCV HSV ranges: H 0-180, S 0-255, V 0-255.
 */
public class BallColorPipeline implements VisionProcessor {

    // ── HSV thresholds ────────────────────────────────────────────────────────
    public static double GREEN_H  = 80;
    public static double PURPLE_H = 130;
    public static double H_RANGE  = 25;   // each color matches H ± H_RANGE
    // Saturation and value minimums shared by both colors (rejects grey/white).
    public static double SAT_MIN = 60;
    public static double VAL_MIN = 60;
    // Minimum matching pixels in a slot to register as a detection.
    // Raise this if you get false positives; lower it if detection misses balls.
    public static int MIN_PIXELS = 100;
    // ─────────────────────────────────────────────────────────────────────────

    /** Hardware map name for the downward-facing ball camera. */
    public static final String CAMERA_NAME = "Webcam 1";

    private final BallColor[] detectedColors = {
            BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN
    };

    // Reused across frames to avoid per-frame allocation.
    private final Mat hsv        = new Mat();
    private final Mat greenMask  = new Mat();
    private final Mat purpleMask = new Mat();

    // Debug stats — populated each frame, read by the calibration OpMode.
    private final int[]      greenCounts  = new int[3];
    private final int[]      purpleCounts = new int[3];
    // centerHSV[slot] = {H, S, V} sampled from the centre of that slot.
    private final double[][] centerHSV    = new double[3][3];

    private final VisionPortal portal;

    public BallColorPipeline(HardwareMap hardwareMap) {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(this)
                .setCameraResolution(new android.util.Size(640, 480))
                .setLiveViewContainerId(0) // disable live view — required when multiple VisionPortals are active
                .build();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        int slotWidth = frame.width() / 3;

        for (int i = 0; i < 3; i++) {
            // Sub-matrix view of this slot — no pixel copy, just a window into hsv.
            Mat slot = new Mat(hsv, new Rect(i * slotWidth, 0, slotWidth, frame.height()));

            Core.inRange(slot,
                    new Scalar(GREEN_H  - H_RANGE, SAT_MIN, VAL_MIN),
                    new Scalar(GREEN_H  + H_RANGE, 255,     255),
                    greenMask);
            Core.inRange(slot,
                    new Scalar(PURPLE_H - H_RANGE, SAT_MIN, VAL_MIN),
                    new Scalar(PURPLE_H + H_RANGE, 255,     255),
                    purpleMask);

            int greenPixels  = Core.countNonZero(greenMask);
            int purplePixels = Core.countNonZero(purpleMask);

            greenCounts[i]  = greenPixels;
            purpleCounts[i] = purplePixels;

            // Sample the HSV value at the centre pixel of this slot.
            int cx = slotWidth / 2;
            int cy = frame.height() / 2;
            double[] px = hsv.get(cy, i * slotWidth + cx);
            centerHSV[i] = (px != null) ? px : new double[]{0, 0, 0};

            if (greenPixels > purplePixels && greenPixels > MIN_PIXELS) {
                detectedColors[i] = BallColor.GREEN;
            } else if (purplePixels > greenPixels && purplePixels > MIN_PIXELS) {
                detectedColors[i] = BallColor.PURPLE;
            } else {
                detectedColors[i] = BallColor.UNKNOWN;
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPixel, float scaleCanvasDensity, Object userContext) {}

    /** Snapshot of the latest color classification for all 3 slots. */
    public BallColor[] getDetectedColors() {
        return detectedColors.clone();
    }

    /** Green pixel counts per slot from the last frame. */
    public int[] getGreenCounts()  { return greenCounts.clone(); }

    /** Purple pixel counts per slot from the last frame. */
    public int[] getPurpleCounts() { return purpleCounts.clone(); }

    /** Centre-pixel HSV values per slot: [slot][0]=H, [1]=S, [2]=V. */
    public double[][] getCenterHSV() {
        double[][] copy = new double[3][3];
        for (int i = 0; i < 3; i++) copy[i] = centerHSV[i].clone();
        return copy;
    }

    public void close() {
        portal.close();
    }
}
