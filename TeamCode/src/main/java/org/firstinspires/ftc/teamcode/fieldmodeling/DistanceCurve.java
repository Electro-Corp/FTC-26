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
    public static final String FILE_PATH = "/sdcard/LogParams-Distance.txt";

    private final List<DistanceDataPoint> points = new ArrayList<>();

    public DistanceCurve() {}

    public DistanceCurve(JsonObject wrapper) {
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

    public static DistanceCurve read() {
        try (Reader r = new FileReader(FILE_PATH)) {
            return new DistanceCurve(JsonParser.parseReader(r).getAsJsonObject());
        } catch (Exception e) {
            return new DistanceCurve();
        }
    }

    public void write() {
        try (Writer w = new FileWriter(FILE_PATH)) {
            w.write(toJSON().toString());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
