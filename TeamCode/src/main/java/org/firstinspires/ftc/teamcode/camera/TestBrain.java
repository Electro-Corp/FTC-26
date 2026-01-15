package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

public class TestBrain extends AprilTagBrain{

    public TestBrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public AprilTagDetection getClosestTag(){
        ArrayList<AprilTagDetection> tags = this.getVisibleTags();

        if(tags.isEmpty()){
            return null;
        }else if(tags.size() == 1){
            return tags.get(0);
        }

        int indexOfClose = 0;
        double sizeOfClosest = Integer.MAX_VALUE;

        for(int i = 0; i < tags.size(); i++){
            if (tags.get(i).ftcPose.range < sizeOfClosest) {
                sizeOfClosest = tags.get(i).ftcPose.range;
                indexOfClose = i;
            }
        }

        return tags.get(indexOfClose);
    }

    public AprilTagDetection getTagID(int id){
        ArrayList<AprilTagDetection> tags = this.getVisibleTags();

        for(int i = 0; i < tags.size(); i++){
            if (tags.get(i).id == id) {
                return tags.get(i);
            }
        }

        return null;
    }

    public boolean isIdOnScreen(int id){
        for(AprilTagDetection tag : this.getVisibleTags())
            if(tag.id == id) return true;

        return false;
    }

    public AprilTagPoseFtc getCloestTagPose(){
        ArrayList<AprilTagDetection> tags = this.getVisibleTags();

        if(tags.isEmpty()){
            return null;
        }else if(tags.size() == 1){
            return tags.get(0).ftcPose;
        }

        int indexOfClose = 0;
        double sizeOfClosest = Integer.MAX_VALUE;

        for(int i = 0; i < tags.size(); i++){
            if (tags.get(i).ftcPose.range < sizeOfClosest) {
                sizeOfClosest = tags.get(i).ftcPose.range;
                indexOfClose = i;
            }
        }

        return tags.get(indexOfClose).ftcPose;
    }

    public int getClosestTagID(){
        return getClosestTag().id;
    }
}
