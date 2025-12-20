package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.google.gson.JsonObject;

public class DataPoint {
    public double posX, posY, heading, speed;

    public DataPoint(double posX, double posY, double heading, double speed) {
        this.posX = posX;
        this.posY = posY;
        this.heading = heading;
        this.speed = speed;
    }

    public JsonObject toJSON() {
        JsonObject obj = new JsonObject();
        obj.addProperty("posX", posX);
        obj.addProperty("posY", posX);
        obj.addProperty("heading", heading);
        obj.addProperty("speed", speed);

        return obj;
    }
}
