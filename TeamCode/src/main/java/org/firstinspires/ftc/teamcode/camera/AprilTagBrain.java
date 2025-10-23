package org.firstinspires.ftc.teamcode.camera;

import android.graphics.Point;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// We need both for some reason
import org.firstinspires.ftc.robotcore.external.android.util.Size;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class AprilTagBrain {
    public class DesertPoint{
        public double x, y;
        public DesertPoint(double x, double y){
            this.x = x;
            this.y = y;
        }
    }
    public class DesertTag{
        public int id;
        public DesertPoint position;

        public DesertTag(int id, DesertPoint point) {
            this.id = id;
            this.position = point;
        }
    }

    public DesertTag[] knownTags = {
            new DesertTag(24, new DesertPoint(126,130)),
            new DesertTag(20, new DesertPoint(15,130))
    };

    private final Size res = new Size(800, 600);
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public AprilTagBrain(HardwareMap hardwareMap){
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        CameraCalibrationIdentity cci = new CameraCalibrationIdentity() {
            @Override
            public boolean isDegenerate() {
                return false;
            }
        };
        float co[] = {0.085871f, -0.212438f, 0, 0, 0.0803528f, 0, 0, 0};
        /*
            Alt calib:
            FX FY 643.474
            OCX 448.969 OCY 315.204
            K1 0.0997 K2 -0.255 K3 0.120
         */
        CameraCalibration calibration = new CameraCalibration(cci, new Size(res.getWidth(), res.getWidth()), 638.756f, 638.756f, 447.895f,319.37f, co, false, false);
        aprilTagProcessor.init(res.getWidth(), res.getHeight(), calibration);
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(res.getWidth(), res.getHeight()))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
    }

    public ArrayList<AprilTagDetection> getVisibleTags(){
        return aprilTagProcessor.getDetections();
    }

    public DesertPoint getCurrentPositionRelativeToCenterOfBackWall(){
        ArrayList<DesertPoint> foundPoints = new ArrayList<DesertPoint>();
        for (AprilTagDetection tag : getVisibleTags()) {
            // Look for a tag
            DesertTag foundTag = findKnownTag(tag.id);
            if(foundTag != null){
                // With the data of our pose and its world position,
                // find out where WE are
                double x = foundTag.position.x - (tag.ftcPose.y * Math.cos(tag.ftcPose.bearing));
                double y = foundTag.position.y - (tag.ftcPose.y * Math.sin(tag.ftcPose.bearing));
                DesertPoint curPoint = new DesertPoint(x,y);
                foundPoints.add(curPoint);
            }
        }
        double avgX = 0.0, avgY = 0.0;
        for(DesertPoint p : foundPoints){
            avgX += p.x;
            avgY += p.y;
        }
        DesertPoint myGuess = new DesertPoint(avgX / foundPoints.size(),avgY / foundPoints.size());
        return myGuess;
    }

    private DesertTag findKnownTag(int i){
        for(DesertTag tag : knownTags){
            if(tag.id == i) return tag;
        }
        return null;
    }

    public float getFPS(){
        return visionPortal.getFps();
    }
}
