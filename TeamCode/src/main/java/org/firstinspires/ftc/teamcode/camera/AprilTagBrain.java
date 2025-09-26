package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// We need both for some reason
import org.firstinspires.ftc.robotcore.external.android.util.Size;

import java.util.ArrayList;
import java.util.List;

public class AprilTagBrain {
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

    public float getFPS(){
        return visionPortal.getFps();
    }
}
