package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.Reader;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DistanceCurve {
    /** Default file path: single-shot calibration curve. */
    public static final String FILE_PATH = "/sdcard/LogParams-Distance.txt";

    /** Three-shot calibration curve (gamepad2.right_bumper "fire all three" flow). */
    public static final String THREE_SHOT_FILE_PATH = "/sdcard/LogParams-Distance-ThreeShot.txt";

    /** File path this instance reads/writes. Defaults to {@link #FILE_PATH}. */
    private final String filePath;

    private final List<DistanceDataPoint> points = new ArrayList<>();

    public DistanceCurve() {
        this(FILE_PATH);
    }

    public DistanceCurve(String filePath) {
        this.filePath = filePath;
    }

    public DistanceCurve(JsonObject wrapper) {
        this(wrapper, FILE_PATH);
    }

    public DistanceCurve(JsonObject wrapper, String filePath) {
        this.filePath = filePath;
        JsonArray array = wrapper.getAsJsonArray("data");
        if (array == null) return;
        for (JsonElement e : array) {
            JsonObject o = e.getAsJsonObject();
            points.add(new DistanceDataPoint(
                    o.get("distance").getAsDouble(),
                    o.get("speed").getAsDouble()));
        }
        sortByDistance();
    }

    public void addPoint(DistanceDataPoint p) {
        points.add(p);
        sortByDistance();
    }

    /** Removes every logged sample. Caller is responsible for persisting via write(). */
    public void clear() {
        points.clear();
    }

    private void sortByDistance() {
        Collections.sort(points, Comparator.comparingDouble(p -> p.distance));
    }

    public int size() {
        return points.size();
    }

    public List<DistanceDataPoint> getPoints() {
        return points;
    }

    /** Linear interpolation, clamped at the curve endpoints. Returns 0 if no data. */
    public double getSpeedAtDistance(double distance) {
        if (points.isEmpty()) return 0;
        if (points.size() == 1) return points.get(0).speed;
        if (distance <= points.get(0).distance) return points.get(0).speed;
        if (distance >= points.get(points.size() - 1).distance) {
            return points.get(points.size() - 1).speed;
        }
        for (int i = 1; i < points.size(); i++) {
            DistanceDataPoint hi = points.get(i);
            if (distance <= hi.distance) {
                DistanceDataPoint lo = points.get(i - 1);
                double t = (distance - lo.distance) / (hi.distance - lo.distance);
                return lo.speed + t * (hi.speed - lo.speed);
            }
        }
        return points.get(points.size() - 1).speed;
    }

    public JsonObject toJSON() {
        JsonArray array = new JsonArray();
        for (DistanceDataPoint p : points) array.add(p.toJSON());
        JsonObject wrapper = new JsonObject();
        wrapper.add("data", array);
        return wrapper;
    }

    /** Default-path read: loads the single-shot curve from {@link #FILE_PATH}. */
    public static DistanceCurve read() {
        return read(FILE_PATH);
    }

    /**
     * Read a curve from an arbitrary file path. Used to maintain multiple
     * curves (e.g. single-shot vs three-shot) in the same OpMode. If the file
     * is missing or unreadable, returns an empty curve bound to that path so
     * subsequent write() calls go to the right place.
     */
    public static DistanceCurve read(String filePath) {
        try (Reader r = new FileReader(filePath)) {
            return new DistanceCurve(new JsonParser().parse(r).getAsJsonObject(), filePath);
        } catch (Exception e) {
            return new DistanceCurve(filePath);
        }
    }

    public void write() {
        try (Writer w = new FileWriter(filePath)) {
            w.write(toJSON().toString());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
