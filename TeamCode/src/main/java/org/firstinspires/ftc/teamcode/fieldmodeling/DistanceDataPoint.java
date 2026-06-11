package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.google.gson.JsonObject;

public class DistanceDataPoint {
    public final double distance;
    public final double speed;

    public DistanceDataPoint(double distance, double speed) {
        this.distance = distance;
        this.speed = speed;
    }

    public JsonObject toJSON() {
        JsonObject o = new JsonObject();
        o.addProperty("distance", distance);
        o.addProperty("speed", speed);
        return o;
    }
}
