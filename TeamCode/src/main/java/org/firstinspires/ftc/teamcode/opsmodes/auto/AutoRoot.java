package org.firstinspires.ftc.teamcode.opsmodes.auto;

import android.graphics.Point;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public abstract class AutoRoot extends LinearOpMode {
    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private Intake intake;
    private Shooter shooter;

    private Thread shooterThread;

    private Shooter.BallColor pattern[] = new Shooter.BallColor[3];
    private int currentIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();

        waitForStart();

        TrajectoryActionBuilder initTurn = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-18, -18))
                .turn(ang(-45));
        runTrajectory(initTurn);

        shootThree();

        intake.go();

        initTurn = drive.actionBuilder(drive.localizer.getPose())
                .turn(ang(180))
                .strafeTo(new Vector2d(-18 - 9, -18))
                .strafeTo(new Vector2d(-18 - 9, 0));
        runTrajectory(initTurn);

        intake.stop();

        initTurn = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-18 - 9, -5))
                .turn(ang(-180));
        runTrajectory(initTurn);

        align(id);

        shootThree();

        shooter.stopShooterThread();
    }

    // Litearlly becuase im lazy
    double ang(double a){
        return Math.toRadians(a);
    }

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooterThread = new Thread(shooter);
        shooterThread.start();
    }

    private void waitForShooter(){
        // Block until shooter is done shooting or force stopped
        while(shooter.getState() != Shooter.ShooterState.STOPPED && !isStopRequested()){
            telemetry.addLine("Waiting for shooter to be done..");
            telemetry.addData("Speed", shooter.getVelocity());
            telemetry.addData("State", shooter.getState());
            telemetry.addData("Shooter Thread is alive", shooterThread.isAlive());
            telemetry.update();
        }
    }

    private void align(int tagId){
        AprilTagDetection tag = tBrain.getTagID(tagId); // Only Red tag right now
        if (tag != null) {
            AprilTagPoseFtc tagPose = tag.ftcPose;

            TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                    .turn(Math.toRadians(tagPose.bearing));

            runTrajectory(trajectory);
        }
    }

    private void runTrajectory(TrajectoryActionBuilder t){
        telemetry.addLine("Running trajectory");
        telemetry.update();
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }

    private void shootNext(){
        if(!shooter.shootColorFar(pattern[currentIndex])){
            // it didn't find the color. . .
            // run the intake to try to put the
            // balls over the color sensor
            intake.go();
            sleep(500); // don't waste *too* much time
            intake.stop();
            shooter.shootColorFar(pattern[currentIndex]);
        }
        if(currentIndex == 2) currentIndex = 0;
        else currentIndex++;
        waitForShooter();
    }

    private void shootThree(){
        for(int i = 0; i < 3; i++){
            shootNext();
        }
    }

    private void generatePattern(int oId){
        // take the obselisk id and figure out what
        // it means. . . .
        switch(oId){
            case 21:
                pattern[0] = Shooter.BallColor.GREEN;
                pattern[1] = Shooter.BallColor.PURPLE;
                pattern[2] = Shooter.BallColor.PURPLE;
                break;
            case 22:
                pattern[0] = Shooter.BallColor.PURPLE;
                pattern[1] = Shooter.BallColor.GREEN;
                pattern[2] = Shooter.BallColor.PURPLE;
                break;
            case 23:
                pattern[0] = Shooter.BallColor.PURPLE;
                pattern[1] = Shooter.BallColor.PURPLE;
                pattern[2] = Shooter.BallColor.GREEN;
                break;
            default:
                // its not from the obselisk...
                break;
        }
    }


    protected abstract int getTargetTag();
}
