package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensorCalib extends LinearOpMode {
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor midColor;
    private NormalizedColorSensor rightColor;
    @Override
    public void runOpMode() throws InterruptedException {
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        midColor = hardwareMap.get(NormalizedColorSensor.class, "midColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            NormalizedRGBA lColors = leftColor.getNormalizedColors();
            NormalizedRGBA mColors = midColor.getNormalizedColors();
            NormalizedRGBA rColors = rightColor.getNormalizedColors();

            telemetry.addLine("[SENSOR] [R] [G] [B]");
            telemetry.addData("left", "%.3f %.3f %.3f", lColors.red, lColors.green, lColors.blue);
            telemetry.addData("mid", "%.3f %.3f %.3f", mColors.red, mColors.green, mColors.blue);
            telemetry.addData("right", "%.3f %.3f %.3f", rColors.red, rColors.green, rColors.blue);
        }
    }
}
