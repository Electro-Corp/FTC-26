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

public abstract class AutoRoot extends LinearOpMode implements Runnable {
    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private Intake intake;
    private Shooter shooter;

    private Thread shooterThread, thisTeleThread;

    private String readOrStatic = "NUN";

    private Shooter.BallColor pattern[] = new Shooter.BallColor[3];
    private int currentIndex = 0, selectedId = -1;

    public void run(){
        while(!isStopRequested()){
            updateTele();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();
        thisTeleThread.start();


        waitForStart();

        shooter.readColors();


        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(-55, 0))
                .turn(ang(50));
        runTrajectory(traj);

        for(int i = 21; i <= 23; i++){
            if(tBrain.getTagID(i) != null){
                generatePattern(i);
            }
            sleep(250);
        }

        if(pattern.length < 1){
            readOrStatic = "STATIC";
            // its not from the obselisk...
            // just do 21
            pattern[0] = Shooter.BallColor.GREEN;
            pattern[1] = Shooter.BallColor.PURPLE;
            pattern[2] = Shooter.BallColor.PURPLE;
        }else{
            readOrStatic = "READ";
        }

        intake.go();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .turn(ang(-50));
        runTrajectory(traj);

        shootThree();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .turn(ang(-200));
        runTrajectory(traj);

        traj = drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(new Vector2d(-90, 0));
        runTrajectory(traj);

        //intake.stop();

        /*intake.go();

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

        shootThree();*/

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

        thisTeleThread = new Thread(this);

        shooterThread.start();
        shooter.readColorsOnce = true;
    }

    private void waitForShooter(){
        // Block until shooter is done shooting or force stopped
        while(shooter.getState() != Shooter.ShooterState.STOPPED && !isStopRequested()){
           // block main thread
        }
    }

    private void updateTele(){
        // Output pattern
        if(pattern[0] != null)
            telemetry.addData("Pattern", "%d: %s %s %s", selectedId, pattern[0].toString(), pattern[1].toString(), pattern[2].toString());
        telemetry.addData("Static or Read", readOrStatic);
        telemetry.addData("Static Loaded",  "%s %s %s", shooter.loadedColors[0].toString(), shooter.loadedColors[1].toString(), shooter.loadedColors[2].toString());
        telemetry.addData("Live Loaded",  "%s %s %s", Shooter.whatColor(shooter.getLeftColor()).toString(), Shooter.whatColor(shooter.getMidColor()).toString(), Shooter.whatColor(shooter.getRightColor()).toString());
        telemetry.addData("Next fire index", currentIndex);
        telemetry.addData("Speed", shooter.getVelocity());
        telemetry.addData("State", shooter.getState());
        telemetry.addData("Shooter Thread is alive", shooterThread.isAlive());

        telemetry.update();
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
        Action currentAction = t.build();
        Actions.runBlocking(currentAction);
    }

    private void shootNext(){
        if(!shooter.shootColorNear(pattern[currentIndex])){
            if(shooter.secLastFir != -1) {
                int val = 3 - (shooter.lastFired + shooter.secLastFir);
                switch (val) {
                    case 0:
                        shooter.setShootSpecific(true, false, false);
                        shooter.shootNear();
                        break;
                    case 1:
                        shooter.setShootSpecific(false, true, false);
                        shooter.shootNear();
                        break;
                    case 2:
                        shooter.setShootSpecific(false, false, true);
                        shooter.shootNear();
                        break;
                    default:
                        shooter.shootColorNear(pattern[currentIndex]);
                }
                shooter.lastFired = -1;
                shooter.secLastFir = -2;
            }else{
                shooter.shootColorNear(pattern[currentIndex]);
            }
        }
        if(currentIndex == 2) currentIndex = 0;
        else currentIndex++;
        // Block until ball is fired
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
                selectedId = oId;
                pattern[0] = Shooter.BallColor.GREEN;
                pattern[1] = Shooter.BallColor.PURPLE;
                pattern[2] = Shooter.BallColor.PURPLE;
                break;
            case 22:
                selectedId = oId;
                pattern[0] = Shooter.BallColor.PURPLE;
                pattern[1] = Shooter.BallColor.GREEN;
                pattern[2] = Shooter.BallColor.PURPLE;
                break;
            case 23:
                selectedId = oId;
                pattern[0] = Shooter.BallColor.PURPLE;
                pattern[1] = Shooter.BallColor.PURPLE;
                pattern[2] = Shooter.BallColor.GREEN;
                break;
            default:
                break;
        }
    }


    protected abstract int getTargetTag();
}
