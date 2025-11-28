package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="ColorCalib")
public class ColorSensorCalib extends LinearOpMode {
    private NormalizedColorSensor leftColor;
    private NormalizedColorSensor midColor;
    private NormalizedColorSensor rightColor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftColor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        midColor = hardwareMap.get(NormalizedColorSensor.class, "midColor");
        rightColor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        float gain = 27.0f;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            leftColor.setGain(gain);
            midColor.setGain(gain);
            rightColor.setGain(gain);

            if(gamepad1.dpad_up) gain += 0.5f;
            if(gamepad1.dpad_down) gain -= 0.5f;


            NormalizedRGBA lColors = leftColor.getNormalizedColors();
            NormalizedRGBA mColors = midColor.getNormalizedColors();
            NormalizedRGBA rColors = rightColor.getNormalizedColors();

            telemetry.addLine("[SENSOR] [R] [G] [B]");
            telemetry.addData("Gain", gain);
            telemetry.addData("left", "%.3f %.3f %.3f", lColors.red, lColors.green, lColors.blue);
            telemetry.addData("mid", "%.3f %.3f %.3f", mColors.red, mColors.green, mColors.blue);
            telemetry.addData("right", "%.3f %.3f %.3f", rColors.red, rColors.green, rColors.blue);

            telemetry.addLine("My predictions:");
            telemetry.addData("Left", whatColor(lColors));
            telemetry.addData("Mid", whatColor(mColors));
            telemetry.addData("Right", whatColor(rColors));

            telemetry.update();
        }
    }

    private String whatColor(NormalizedRGBA color){
        if(color.green > 0.600){
            if(color.blue > 0.800){
                return "Purple";
            }
            return "Green";
        }else{
            return "idk";
        }
    }
}
