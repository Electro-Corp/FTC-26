package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import java.util.List;

public class AprilTagBrain {

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
        float co[] = { -0.0347492f, -0.0148858f,0, 0, -0.0072575f,  0.0121186f, -0.156374f, 0};
        CameraCalibration calibration = new CameraCalibration(cci, new Size(640, 480), 499.542f, 499.542f, 341.04f,225.8f, co, false, false);
        aprilTagProcessor.init(640, 480, calibration);
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTagProcessor).build();
    }

    public List<AprilTagDetection> getVisibleTags(){
        return aprilTagProcessor.getDetections();
    }
}
