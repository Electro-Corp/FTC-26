package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

@TeleOp
public class LimelightAlign extends LinearOpMode {

    // Limelight
    Limelight3A limelight;

    // PedroPath
    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();

        while(!isStopRequested()){
            LLResult res = limelight.getLatestResult();

            if(res != null && res.isValid()){
                double tX = res.getTx();

                follower.followPath(follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), follower.getPose()))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), follower.getPose().getHeading() + Math.toRadians(tX))
                        .build());

                telemetry.addData("Target X", tX);
//                    telemetry.addData("Target Y", res.getTy());
//                    telemetry.addData("Target Area", res.getTa());
            }
            follower.update();
            telemetry.update();
        }

    }

    private void initHardware(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);

        // Limelight config
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }
}