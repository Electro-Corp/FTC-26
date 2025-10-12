package org.firstinspires.ftc.teamcode.opsmodes;

/*
    Very Slow Alignment, just look at MainOPsMode for the real stuff
 */
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name="RRAlign", group="TeleOp")
public class RoadRunnerAlign extends LinearOpMode {

    private double ROT_TOL = 2.0;
    private double incAmount = 0.1;
    private int curPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TestBrain tBrain = new TestBrain(hardwareMap);

        Pose2d initPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        telemetry.setNumDecimalPlaces(1, 3);

        waitForStart();

        while(opModeIsActive()){
            AprilTagDetection tag = tBrain.getTagID(24);

            Action trajAction = null;

            telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size());
            if(tag != null) {
                AprilTagPoseFtc tagPose = tag.ftcPose;
                if(tagPose != null) {
                    telemetry.addData("Range", tagPose.range);
                    telemetry.addData("Bearing", tagPose.bearing);

                    incAmount = (tagPose.bearing * tagPose.bearing) / 1000;

                    if (tagPose.bearing > 0 + ROT_TOL) {
                        telemetry.addLine("tag.bearing > 0");
                        TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                                .turn(incAmount);
                        trajAction = trajectory.build();
                    } else if (tagPose.bearing < 0 - ROT_TOL) {
                        telemetry.addLine("tag.bearing < 0");
                        TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                                .turn(-incAmount);
                        trajAction = trajectory.build();
                    } else {

                    }

                    telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", tagPose.x, tagPose.y, tagPose.z);
                }
            }else{
                telemetry.addLine("No Pose available");
            }

            if (trajAction != null) {
                Actions.runBlocking(trajAction);
            }
            telemetry.addData("Target Pos", curPos);
            telemetry.addData("FPS", (int)tBrain.getFPS());
            telemetry.update();
        }
    }
}
