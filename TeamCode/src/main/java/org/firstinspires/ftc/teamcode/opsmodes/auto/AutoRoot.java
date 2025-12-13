package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
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
    private ColorSensors colorSensors;

    private Thread shooterThread, thisTeleThread;

    private String readOrStatic = "NUN";

    private Pattern pattern;
    private int currentIndex = 0;
    private boolean[] fired = new boolean[3];

    public void run(){
        while(!isStopRequested()){
            updateTele();
        }
        shooter.stopShooterThread();
    }

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter(hardwareMap, colorSensors, true);

        shooter.SPINNER_SPEED_NEAR = -1280;

        shooterThread = new Thread(shooter);
        thisTeleThread = new Thread(this);

        shooterThread.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();
        thisTeleThread.start();

        waitForStart();

        shooter.spinUp(false);

        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .lineToXConstantHeading(-42)
                .turn(ang(50));
        runTrajectory(traj);

        pattern = readObelisk();

        intake.go();

        traj = traj.endTrajectory().fresh()
                .turn(ang(-52));
        runTrajectory(traj);

        //align(getTargetTag());

        shootThree();

        intake.setSpeed(-0.88);

        intake.go();

        traj = traj.endTrajectory().fresh()
                //.lineToXConstantHeading(-50)
                .turn(ang((125)))
                .lineToYConstantHeading(-35 * getInvert());
        runTrajectory(traj);

        shooter.spinUp(false);

        traj = traj.endTrajectory().fresh()
                .lineToYConstantHeading(-2 * getInvert())
                .turn(ang(-(125)));
                //.lineToXConstantHeading(-30);
        runTrajectory(traj);

        intake.stop();

        //align(getTargetTag());

        shootThree();

        traj = traj.endTrajectory().fresh()
                .turn(ang(-120))
                .lineToYConstantHeading(-30 * getInvert());
        runTrajectory(traj);

        shooter.stopShooterThread();
    }

    private Pattern readObelisk() {
        for(int i = 21; i <= 23; i++){
            if(tBrain.getTagID(i) != null){
                return Pattern.fromNum(i);
            }
            sleep(250);
        }
        return Pattern.fromNum(-1);
    }

    // Litearlly becuase im lazy
    double ang(double a){
        return Math.toRadians(getInvert() * a);
    }

    // Find an unfired ball that matches the requested color
    private int findIndexForColor(BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (!fired[i] && shooter.getLoadedColors()[i] == color) {
                return i;
            }
        }
        return -1;
    }

    // Fallback: just pick any remaining unfired ball
    private int findAnyUnfiredIndex() {
        for (int i = 0; i < 3; i++) {
            if (!fired[i]) {
                return i;
            }
        }
        return -1;
    }

    private void waitForShooter(){
        // Block until shooter is done shooting or force stopped
        while(shooter.getState() != Shooter.ShooterState.STOPPED && !isStopRequested()){
           // block main thread
        }
    }

    private void updateTele(){
        // Output pattern
        BallColor[] loadedColors = shooter.getLoadedColors();
        if(pattern != null)
            telemetry.addData("Pattern", pattern.toString());
        telemetry.addData("Static or Read", readOrStatic);
        telemetry.addData("Static Loaded",  "%s %s %s", loadedColors[0], loadedColors[1], loadedColors[2]);
        telemetry.addData("Live Loaded",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
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

    private void shootNext() {
        BallColor targetColor = pattern.getColorAtIndex(currentIndex);

        // First try to find a ball that matches the pattern color
        int indexToShoot = findIndexForColor(targetColor);

        // If no matching color is left, just shoot any remaining ball
        if (indexToShoot == -1) {
            indexToShoot = findAnyUnfiredIndex();
        }

        // If nothing left, we are done
        if (indexToShoot == -1) {
            return;
        }

        // Tell the shooter which position to fire
        switch (indexToShoot) {
            case 0:
                shooter.setShootSpecific(true, false, false);
                break;
            case 1:
                shooter.setShootSpecific(false, true, false);
                break;
            case 2:
                shooter.setShootSpecific(false, false, true);
                break;
            default:
                return;
        }

        shooter.shootNear();
        fired[indexToShoot] = true;

        // Advance to next pattern index
        if (currentIndex == 2) {
            currentIndex = 0;
        } else {
            currentIndex++;
        }

        // Block until ball is actually shot
        waitForShooter();
    }

    private void shootThree(){
        // Read
        shooter.updateLoadedColors();
        for(int i = 0; i < 3; i++){
            shootNext();
            // Poor hack
            //if(i == 0) shooter.SPINNER_SPEED_NEAR = -1300;
            //if(i > 0) shooter.SPINNER_SPEED_NEAR = -1350;
            shooter.spinUp(false);
            sleep(200);
            while(!(Math.abs(shooter.SPINNER_SPEED_NEAR) - 10 < shooter.getVelocity() && Math.abs(shooter.SPINNER_SPEED_NEAR) + 10 > shooter.getVelocity())){}
        }
        // Reset
        fired = new boolean[3];
        shooter.stopShoot();
    }



    protected abstract int getTargetTag();

    protected abstract int getInvert();
}
