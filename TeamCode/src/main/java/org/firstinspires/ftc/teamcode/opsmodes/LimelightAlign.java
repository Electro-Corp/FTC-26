package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

@TeleOp
public class LimelightAlign extends LinearOpMode {

    // Limelight
    Limelight3A limelight;

    // Roadrunner
    MecanumDrive mecanumDrive;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();

        while(!isStopRequested()){
            LLResult res = limelight.getLatestResult();

            if(res != null && res.isValid()){
                double tX = res.getTx();

                TrajectoryActionBuilder trajectory = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                        .turn(Math.toRadians(tX));
                //runTrajectory(trajectory);

                telemetry.addData("Target X", tX);
//                    telemetry.addData("Target Y", res.getTy());
//                    telemetry.addData("Target Area", res.getTa());
            }

            telemetry.update();
        }

    }

    private void initHardware(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));

        // Limelight config
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    private void runTrajectory(TrajectoryActionBuilder t){
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }
}
