package org.firstinspires.ftc.teamcode.camera;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

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
 * The frame is split into 3 equal horizontal strips:
 *   slot 0 = left, slot 1 = centre (inferred), slot 2 = right
 * matching the shooter kicker layout (left, middle, right).
 *
 * Only slots 0 and 2 are directly read; slot 1 is inferred from the constraint
 * that there are always exactly 1 green and 2 purples loaded.
 *
 * OpenCV HSV ranges: H 0-180, S 0-255, V 0-255.
 */
public class BallColorPipeline implements VisionProcessor {

    // ── HSV thresholds ────────────────────────────────────────────────────────
    // OpenCV HSV scale: H 0-180, S 0-255, V 0-255 (hue is circular: 0 == 180).
    public static double GREEN_H     = 120;  // green balls read ~120
    public static double GREEN_RANGE = 25;
    // Purple wraps around the 0/180 boundary (reads ~20 and ~160).
    // We split into [0, PURPLE_RANGE] and [180-PURPLE_RANGE, 180] then OR them.
    public static double PURPLE_RANGE = 35;
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
    private final Mat hsv          = new Mat();
    private final Mat greenMask    = new Mat();
    private final Mat purpleMask   = new Mat();
    private final Mat purpleMask2  = new Mat();

    // Debug stats — populated each frame, read by the calibration OpMode.
    private final int[]      greenCounts  = new int[3];
    private final int[]      purpleCounts = new int[3];
    // centerHSV[slot] = {H, S, V} sampled from the centre of that slot.
    private final double[][] centerHSV    = new double[3][3];

    private final VisionPortal portal;

    /** Standard constructor — live view disabled (required when multiple portals are active). */
    public BallColorPipeline(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * @param enableLiveView true to show the camera feed on the Driver Hub.
     *                       Pass false (or use the single-arg constructor) when
     *                       another VisionPortal is also active, e.g. TestBrain in auto.
     */
    public BallColorPipeline(HardwareMap hardwareMap, boolean enableLiveView) {
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .addProcessor(this)
                .setCameraResolution(new android.util.Size(640, 480));
        if (!enableLiveView) {
            builder.setLiveViewContainerId(0);
        }
        portal = builder.build();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        int slotWidth = frame.width() / 3;

        // Only read left (0) and right (2) — middle is blocked by robot hardware.
        int cx, cy;
        int[] outerSlots = {0, 2};
        for (int i : outerSlots) {
            Mat slot = new Mat(hsv, new Rect(i * slotWidth, 0, slotWidth, frame.height()));

            Core.inRange(slot,
                    new Scalar(GREEN_H - GREEN_RANGE, SAT_MIN, VAL_MIN),
                    new Scalar(GREEN_H + GREEN_RANGE, 255,     255),
                    greenMask);
            // Purple wraps around the hue boundary: catch [0, RANGE] and [180-RANGE, 180].
            Core.inRange(slot,
                    new Scalar(0,                    SAT_MIN, VAL_MIN),
                    new Scalar(PURPLE_RANGE,         255,     255),
                    purpleMask);
            Core.inRange(slot,
                    new Scalar(180 - PURPLE_RANGE,   SAT_MIN, VAL_MIN),
                    new Scalar(180,                  255,     255),
                    purpleMask2);
            Core.bitwise_or(purpleMask, purpleMask2, purpleMask);

            int greenPixels  = Core.countNonZero(greenMask);
            int purplePixels = Core.countNonZero(purpleMask);

            greenCounts[i]  = greenPixels;
            purpleCounts[i] = purplePixels;

            cx = slotWidth / 2;
            cy = frame.height() / 2;
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

        // Infer from the constraint: always exactly 1 green and 2 purples.
        boolean leftGreen   = detectedColors[0] == BallColor.GREEN;
        boolean rightGreen  = detectedColors[2] == BallColor.GREEN;
        boolean leftPurple  = detectedColors[0] == BallColor.PURPLE;
        boolean rightPurple = detectedColors[2] == BallColor.PURPLE;

        if (leftGreen) {
            detectedColors[1] = BallColor.PURPLE;
            detectedColors[2] = BallColor.PURPLE;
        } else if (rightGreen) {
            detectedColors[0] = BallColor.PURPLE;
            detectedColors[1] = BallColor.PURPLE;
        } else if (leftPurple && rightPurple) {
            detectedColors[1] = BallColor.GREEN;
        } else {
            detectedColors[1] = BallColor.UNKNOWN;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPixel, float scaleCanvasDensity,
                            Object userContext) {
        float slotW    = onscreenWidth / 3f;
        float pad      = 8  * scaleCanvasDensity;
        float stroke   = 5  * scaleCanvasDensity;
        float textLg   = 26 * scaleCanvasDensity;
        float textSm   = 18 * scaleCanvasDensity;

        Paint fillPaint = new Paint();
        fillPaint.setStyle(Paint.Style.FILL);

        Paint borderPaint = new Paint();
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(stroke);

        Paint textPaint = new Paint();
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setColor(Color.WHITE);

        String[] slotLabels   = {"LEFT",   "CENTER\n(inferred)", "RIGHT"};
        boolean[] inferred    = {false, true, false};

        for (int i = 0; i < 3; i++) {
            float left  = i * slotW;
            float right = left + slotW;

            int col = colorFor(detectedColors[i]);

            // Semi-transparent fill so the camera image is still visible.
            fillPaint.setColor(col);
            fillPaint.setAlpha(70);
            canvas.drawRect(left, 0, right, onscreenHeight, fillPaint);

            // Solid border.
            borderPaint.setColor(col);
            borderPaint.setAlpha(255);
            canvas.drawRect(left, 0, right, onscreenHeight, borderPaint);

            // Slot name.
            textPaint.setTextSize(textLg);
            canvas.drawText(slotLabels[i].split("\n")[0], left + pad, textLg + pad, textPaint);
            if (inferred[i]) {
                textPaint.setTextSize(textSm);
                canvas.drawText("(inferred)", left + pad, textLg + pad + textSm + 2, textPaint);
                textPaint.setTextSize(textLg);
            }

            // Detected color name.
            float nameY = inferred[i] ? textLg + textSm + pad * 3 + 4 : textLg * 2 + pad;
            canvas.drawText(detectedColors[i].toString(), left + pad, nameY + textLg, textPaint);

            // Pixel counts for directly-read slots only.
            if (!inferred[i]) {
                textPaint.setTextSize(textSm);
                float countsY = nameY + textLg + pad;
                canvas.drawText("G: " + greenCounts[i],  left + pad, countsY + textSm,          textPaint);
                canvas.drawText("P: " + purpleCounts[i], left + pad, countsY + textSm * 2 + 4,  textPaint);

                // HSV readout.
                double[] h = centerHSV[i];
                canvas.drawText(String.format("H:%.0f S:%.0f V:%.0f", h[0], h[1], h[2]),
                        left + pad, countsY + textSm * 3 + 8, textPaint);
                textPaint.setTextSize(textLg);
            }
        }

        // Dividing lines between slots.
        Paint linePaint = new Paint();
        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setColor(Color.WHITE);
        linePaint.setStrokeWidth(2 * scaleCanvasDensity);
        canvas.drawLine(slotW,     0, slotW,     onscreenHeight, linePaint);
        canvas.drawLine(2 * slotW, 0, 2 * slotW, onscreenHeight, linePaint);
    }

    private static int colorFor(BallColor c) {
        if (c == BallColor.GREEN)  return Color.rgb(0, 210, 0);
        if (c == BallColor.PURPLE) return Color.rgb(150, 0, 255);
        return Color.GRAY;
    }

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
