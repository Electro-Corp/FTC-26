package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class FieldDataPoints {
    private ArrayList<DataPoint> points;

    private DataPoint goal = new DataPoint(60, -66, Math.toRadians(-50), -1300);

    public FieldDataPoints(JsonObject wrapper){
        points = new ArrayList<>();

        JsonArray array = wrapper.get("data").getAsJsonArray();

        // Load array
        for(JsonElement e : array){
            JsonObject object = e.getAsJsonObject();

            double posX = object.get("posX").getAsDouble();
            double posY = object.get("posY").getAsDouble();
            double heading = object.get("heading").getAsDouble();
            double speed = object.get("speed").getAsDouble();

            DataPoint point = new DataPoint(posX, posY, heading, speed);

            points.add(point);
        }
    }

    public FieldDataPoints(){
        points = new ArrayList<>();
    }

    public void addDataPoint(DataPoint point){
        points.add(point);
    }

    public JsonObject toJSON(){
        JsonArray array = new JsonArray();

        for(DataPoint var : points){
            array.add(var.toJSON());
        }

        JsonObject wrapper = new JsonObject();
        wrapper.add("data", array);

        return wrapper;
    }

    private class DataPointWDist{
        public DataPoint dP;
        public double dist;

        public double getDist(){
            return dist;
        }
    }

    public DataPoint getStateAtPose(Pose2d pose) {
        ArrayList<DataPointWDist> distances = new ArrayList<>();

        for (int i = 0; i < points.size(); i++) {
            DataPointWDist d = new DataPointWDist();
            d.dP = points.get(i);
            d.dist = Math.hypot(
                    pose.position.x - points.get(i).posX,
                    pose.position.y - points.get(i).posY
            );
            distances.add(d);
        }

        Collections.sort(distances, Comparator.comparing(DataPointWDist::getDist));

        DataPointWDist d1 = distances.get(0);
        DataPointWDist d2 = distances.get(1);
        DataPointWDist d3 = distances.get(2);

        // Exact match short-circuit
        if (d1.dist == 0) {
            return new DataPoint(
                    pose.position.x,
                    pose.position.y,
                    d1.dP.heading,
                    d1.dP.speed
            );
        }

        double w1 = 1.0 / d1.dist;
        double w2 = 1.0 / d2.dist;
        double w3 = 1.0 / d3.dist;

        double wSum = w1 + w2 + w3;

//        double heading =
//                d1.dP.heading * (w1 / wSum) +
//                        d2.dP.heading * (w2 / wSum) +
//                        d3.dP.heading * (w3 / wSum);

        //double hypot = Math.hypot(goal.posX - pose.position.x, goal.posY - pose.position.y);

        double heading = -Math.atan2(goal.posY - pose.position.y, goal.posX - pose.position.x);

        double speed =
                d1.dP.speed * (w1 / wSum) +
                        d2.dP.speed * (w2 / wSum) +
                        d3.dP.speed * (w3 / wSum);

        return new DataPoint(pose.position.x, pose.position.y, -heading, speed);
    }


    public double getNumOfPoints(){
        return points.size();
    }


}
