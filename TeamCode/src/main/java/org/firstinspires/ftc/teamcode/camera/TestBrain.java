package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class TestBrain extends AprilTagBrain{

    public TestBrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public AprilTagDetection closestTag(){
        List<AprilTagDetection> tags = this.getVisibleTags();

        if(tags.size() == 0){
            return null;
        }

        int indexOfClose = 0;
        double sizeOfClosest = Integer.MAX_VALUE;

        for(int i = 0; i < tags.size(); i++){
            if(tags.get(i).ftcPose != null) {
                if (tags.get(i).ftcPose.range < sizeOfClosest) {
                    sizeOfClosest = tags.get(i).ftcPose.range;
                    indexOfClose = i;
                }
            }
        }

        return tags.get(indexOfClose);
    }
}
