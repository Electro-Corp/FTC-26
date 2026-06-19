package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.camera.BallColorPipeline;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;

/**
 * Calibration OpMode for BallColorPipeline.
 *
 * Point the downward camera at the loaded balls and watch telemetry to verify
 * that each slot detects the correct color. Use the values shown here to tune
 * the HSV thresholds in BallColorPipeline:
 *
 *   GREEN_H_MIN / GREEN_H_MAX   — hue range for green balls  (OpenCV H: 0-180)
 *   PURPLE_H_MIN / PURPLE_H_MAX — hue range for purple balls
 *   SAT_MIN / VAL_MIN           — minimum saturation and value (filters out grey/white)
 *   MIN_PIXELS                  — raise if you see false positives, lower if misses
 *
 * Workflow:
 *   1. Load one green and one purple ball into the robot.
 *   2. Run this OpMode and check "slot N center HSV" for each slot.
 *   3. Set GREEN_H_MIN/MAX to bracket the H value seen from a green ball,
 *      and PURPLE_H_MIN/MAX for the purple ball. Leave ~10 degrees of margin.
 *   4. If a ball is UNKNOWN despite good pixel counts, lower SAT_MIN or VAL_MIN.
 *   5. If you see false positives (wrong color detected), raise MIN_PIXELS.
 */
@TeleOp(name = "Ball Color Calibration")
public class BallColorCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        BallColorPipeline pipeline = new BallColorPipeline(hardwareMap, true);

        telemetry.addLine("Camera initializing...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            BallColor[] colors  = pipeline.getDetectedColors();
            int[]  green        = pipeline.getGreenCounts();
            int[]  purple       = pipeline.getPurpleCounts();
            double[][] hsv      = pipeline.getCenterHSV();

            String[] slotNames = {"left", "center (inferred)", "right"};
            for (int i = 0; i < 3; i++) {
                telemetry.addLine("════ Slot " + i + " (" + slotNames[i] + ") ════");
                telemetry.addData("  Color   ", colors[i]);
                telemetry.addData("  G / P px", "%d / %d", green[i], purple[i]);
                telemetry.addData("  H / S / V", "%.0f / %.0f / %.0f",
                        hsv[i][0], hsv[i][1], hsv[i][2]);
            }

            telemetry.addLine("════ CURRENT THRESHOLDS ════");
            telemetry.addData("Green  H", "%.0f ± %.0f  (%.0f – %.0f)",
                    BallColorPipeline.GREEN_H, BallColorPipeline.GREEN_RANGE,
                    BallColorPipeline.GREEN_H - BallColorPipeline.GREEN_RANGE,
                    BallColorPipeline.GREEN_H + BallColorPipeline.GREEN_RANGE);
            telemetry.addData("Purple H", "wrap ± %.0f  (0–%.0f and %.0f–180)",
                    BallColorPipeline.PURPLE_RANGE,
                    BallColorPipeline.PURPLE_RANGE,
                    180 - BallColorPipeline.PURPLE_RANGE);
            telemetry.addData("Sat min / Val min", "%.0f / %.0f",
                    BallColorPipeline.SAT_MIN, BallColorPipeline.VAL_MIN);
            telemetry.addData("Min pixels", BallColorPipeline.MIN_PIXELS);

            telemetry.update();
        }

        pipeline.close();
    }
}
