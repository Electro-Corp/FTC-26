package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Handles all ball color sensing for the robot. This subsystem manages the three
 * normalized color sensors assigned to the left, middle, and right ball positions,
 * applies consistent gain settings, and provides methods to read and classify each
 * sensor’s detected ball color (green, purple, unknown)
 */
public class ColorSensors {

    private static final float COLOR_GAIN = 30.5f;

    private final NormalizedColorSensor leftColorSensor;
    private final NormalizedColorSensor midColorSensor;
    private final NormalizedColorSensor rightColorSensor;

    public ColorSensors(HardwareMap hardwareMap) {
        leftColorSensor = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        midColorSensor = hardwareMap.get(NormalizedColorSensor.class, "midColor");
        rightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColor");

        leftColorSensor.setGain(COLOR_GAIN);
        midColorSensor.setGain(COLOR_GAIN);
        rightColorSensor.setGain(COLOR_GAIN);
    }

    private NormalizedRGBA readLeftColorRaw() {
        return leftColorSensor.getNormalizedColors();
    }

    private NormalizedRGBA readMidColorRaw() {
        return midColorSensor.getNormalizedColors();
    }

    private NormalizedRGBA readRightColorRaw() {
        return rightColorSensor.getNormalizedColors();
    }

    // Classified colors
    public BallColor readLeftColor() {
        return classifyColor(readLeftColorRaw());
    }

    public BallColor readMidColor() {
        return classifyColor(readMidColorRaw());
    }

    public BallColor readRightColor() {
        return classifyColor(readRightColorRaw());
    }

    /**
     * Read and classify all three at once.
     * Index 0 = left, 1 = mid, 2 = right.
     */
    public BallColor[] readAllColors() {
        BallColor[] result = new BallColor[3];
        result[0] = readLeftColor();
        result[1] = readMidColor();
        result[2] = readRightColor();
        return result;
    }

    /**
     * Classifies a ball's color based on normalized sensor values, identifying it as
     * green, purple, or unknown
     */
    public static BallColor classifyColor(NormalizedRGBA color) {
        if (color.green > 0.100) {
            if (color.blue > 0.100 && color.blue > color.green) {
                return BallColor.PURPLE;
            }
            return BallColor.GREEN;
        } else {
            return BallColor.UNKNOWN;
        }
    }
}
