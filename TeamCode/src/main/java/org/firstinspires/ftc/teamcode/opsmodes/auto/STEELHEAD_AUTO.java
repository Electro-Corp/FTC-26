package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.io.FileWriter;
import java.io.Writer;

@Config
@Autonomous
public class STEELHEAD_AUTO extends LinearOpMode {

    Pose2d initPose = null;
    MecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initPose = new Pose2d(54,-58 * getInvert(), ang(-53));
        drive = new MecanumDrive(hardwareMap, initPose);

        waitForStart();

        Thread.sleep(25000);

        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(35,  -35 * getInvert()), ang(-50));
        runTrajectory(traj);

        dumpPosition();

    }

    public void dumpPosition(){
        Writer logOut;

        try{
            logOut = new FileWriter("/sdcard/end.json");
        } catch (Exception e){
            throw new RuntimeException(e);
        }

        try {
            JsonObject obj = new JsonObject();
            obj.addProperty("posX", drive.localizer.getPose().position.x);
            obj.addProperty("posY", drive.localizer.getPose().position.y);
            obj.addProperty("heading", drive.localizer.getPose().heading.toDouble());
            JsonObject wrapper = new JsonObject();
            wrapper.add("start", obj);

            logOut.write(wrapper.toString());
            logOut.close();
        } catch (Exception e){
            throw new RuntimeException(e);
        }
    }

    public int getInvert(){
        return 1;
    }

    public double ang(double a){
        return Math.toRadians(a * getInvert());
    }

    private void runTrajectory(TrajectoryActionBuilder t){
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }
}
