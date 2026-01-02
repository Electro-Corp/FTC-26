package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.acmerobotics.roadrunner.Pose2d;
import org.junit.Test;

import static org.firstinspires.ftc.robotcore.internal.system.Assert.assertEquals;
import static org.junit.Assert.*;

public class FieldDataPointsTest {

    @Test
    public void getStateAtPose_returnsWeightedAverageOfNearestPoints() {
        FieldDataPoints field = new FieldDataPoints();

        // Three known data points
        field.addDataPoint(new DataPoint(0, 0, 0.0, 1.0));
        field.addDataPoint(new DataPoint(10, 0, 90.0, 2.0));
        field.addDataPoint(new DataPoint(0, 10, 180.0, 3.0));

        // Pose near the center
        Pose2d pose = new Pose2d(3, 3, 0);

        DataPoint result = field.getStateAtPose(pose);

        // Position should match pose
        assertEquals(3.0, result.posX, 1e-6);
        assertEquals(3.0, result.posY, 1e-6);

        // Heading and speed should be reasonable weighted values
        assertTrue(result.heading > 0);
        assertTrue(result.heading < 180);

        assertTrue(result.speed > 1.0);
        assertTrue(result.speed < 3.0);
    }

    @Test
    public void getStateAtPose_exactInterpolation_centerPoint() {
        FieldDataPoints field = new FieldDataPoints();

        field.addDataPoint(new DataPoint(0, 0, 0.0, 1.0));
        field.addDataPoint(new DataPoint(10, 0, 90.0, 2.0));
        field.addDataPoint(new DataPoint(0, 10, 180.0, 3.0));

        Pose2d pose = new Pose2d(5, 5, 0);

        DataPoint result = field.getStateAtPose(pose);

        // Position should exactly match pose
        assertEquals(5.0, result.posX, 1e-9);
        assertEquals(5.0, result.posY, 1e-9);

        // Exact interpolated values based on current implementation
        assertEquals(90, result.heading, 1e-9);
        assertEquals(2, result.speed, 1e-9);
    }

    @Test
    public void getStateAtPose_usesClosestThreeOutOfTen_exactResult() {
        FieldDataPoints field = new FieldDataPoints();

        // --- Closest three (distance = 1) ---
        field.addDataPoint(new DataPoint(5, 6, 0.0, 1.0));
        field.addDataPoint(new DataPoint(6, 5, 90.0, 2.0));
        field.addDataPoint(new DataPoint(5, 4, 180.0, 3.0));

        // --- Far points (should be ignored) ---
        field.addDataPoint(new DataPoint(100, 100, 999, 999));
        field.addDataPoint(new DataPoint(-100, 100, 999, 999));
        field.addDataPoint(new DataPoint(100, -100, 999, 999));
        field.addDataPoint(new DataPoint(-100, -100, 999, 999));
        field.addDataPoint(new DataPoint(50, 0, 999, 999));
        field.addDataPoint(new DataPoint(0, 50, 999, 999));
        field.addDataPoint(new DataPoint(-50, 0, 999, 999));

        Pose2d pose = new Pose2d(5, 5, 0);

        DataPoint result = field.getStateAtPose(pose);

        // Position should match pose exactly
        assertEquals(5.0, result.posX, 1e-9);
        assertEquals(5.0, result.posY, 1e-9);

        // Exact interpolated values from the closest three
        assertEquals(90.0, result.heading, 1e-9);
        assertEquals(2.0, result.speed, 1e-9);
    }

    @Test
    public void getStateAtPose_tenPoints_fivePoses_exactResults() {
        FieldDataPoints field = new FieldDataPoints();

        // ---- (0,0) cluster ----
        field.addDataPoint(new DataPoint(0, 1, 0, 1));
        field.addDataPoint(new DataPoint(1, 0, 90, 2));
        field.addDataPoint(new DataPoint(0, -1, 180, 3));

        // ---- (10,0) cluster ----
        field.addDataPoint(new DataPoint(10, 1, 30, 4));
        field.addDataPoint(new DataPoint(11, 0, 60, 5));
        field.addDataPoint(new DataPoint(10, -1, 90, 6));

        // ---- (0,10) cluster ----
        field.addDataPoint(new DataPoint(0, 11, 45, 7));
        field.addDataPoint(new DataPoint(1, 10, 90, 8));
        field.addDataPoint(new DataPoint(0, 9, 135, 9));

        // ---- Exact match point ----
        field.addDataPoint(new DataPoint(100, 100, 999, 999));

        // ================= Pose 1 =================
        {
            Pose2d pose = new Pose2d(0, 0, 0);
            DataPoint r = field.getStateAtPose(pose);

            assertEquals(0.0, r.posX, 1e-9);
            assertEquals(0.0, r.posY, 1e-9);
            assertEquals(90.0, r.heading, 1e-9);
            assertEquals(2.0, r.speed, 1e-9);
        }

        // ================= Pose 2 =================
        {
            Pose2d pose = new Pose2d(10, 0, 0);
            DataPoint r = field.getStateAtPose(pose);

            assertEquals(10.0, r.posX, 1e-9);
            assertEquals(0.0, r.posY, 1e-9);
            assertEquals(60.0, r.heading, 1e-9);
            assertEquals(5.0, r.speed, 1e-9);
        }

        // ================= Pose 3 =================
        {
            Pose2d pose = new Pose2d(0, 10, 0);
            DataPoint r = field.getStateAtPose(pose);

            assertEquals(0.0, r.posX, 1e-9);
            assertEquals(10.0, r.posY, 1e-9);
            assertEquals(90.0, r.heading, 1e-9);
            assertEquals(8.0, r.speed, 1e-9);
        }
        
        // ================= Pose 4 =================
        // Exact distance match short-circuits interpolation
        {
            Pose2d pose = new Pose2d(100, 100, 0);
            DataPoint r = field.getStateAtPose(pose);

            assertEquals(100.0, r.posX, 1e-9);
            assertEquals(100.0, r.posY, 1e-9);
            assertEquals(999.0, r.heading, 1e-9);
            assertEquals(999.0, r.speed, 1e-9);
        }
    }

}
