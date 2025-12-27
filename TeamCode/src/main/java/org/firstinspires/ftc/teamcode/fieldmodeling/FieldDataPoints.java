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

    public DataPoint getStateAtPose(Pose2d pose){
        ArrayList<DataPoint> closestThree = new ArrayList<>();

        ArrayList<DataPointWDist> distances = new ArrayList<>();

        for(int i = 0; i < points.size(); i++){
            DataPointWDist d = new DataPointWDist();
            d.dP = points.get(i);
            d.dist = Math.sqrt(Math.pow((pose.position.x - points.get(i).posX), 2) + Math.pow((pose.position.y - points.get(i).posY), 2));
        }

        Collections.sort(distances, Comparator.comparing(DataPointWDist::getDist));

        DataPoint one = distances.get(0).dP;
        DataPoint two = distances.get(1).dP;
        DataPoint three = distances.get(2).dP;

        double totalDist = distances.get(0).dist + distances.get(1).dist + distances.get(2).dist;

        double heading = ((one.heading * (distances.get(0).dist / totalDist)) + (two.heading * (distances.get(1).dist / totalDist)) + (three.heading * (distances.get(2).dist / totalDist))) / 3;
        double speed = ((one.speed * (distances.get(0).dist / totalDist)) + (two.speed * (distances.get(1).dist / totalDist)) + (three.speed * (distances.get(2).dist / totalDist))) / 3;

        return new DataPoint(pose.position.x, pose.position.y, heading, speed);
    }

}
