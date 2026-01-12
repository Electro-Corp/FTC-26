package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.fieldmodeling.DataLogger;
import org.firstinspires.ftc.teamcode.fieldmodeling.FieldDataPoints;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kickers;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterCommands;
import org.firstinspires.ftc.teamcode.subsystems.Shooter_New;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@Config
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

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public FieldDataPoints fieldMap;

    public static PIDFCoefficients pid = new PIDFCoefficients(30,0.3,0.5,12);

    public static class PositionsHeadings{
        public double obeliskPos = -20;
        public double obeliskAngle = 0;
    }

    PositionsHeadings posHeadings;

    public void run(){
        while(!isStopRequested()){
            updateAutoThread();
        }
        shooter.stopShooterThread();
    }

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        initPose = new Pose2d(54,-54 * getInvert(), ang(-50 * getInvert()));
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter(hardwareMap, colorSensors, true);
        fieldMap = DataLogger.read();

        posHeadings = new PositionsHeadings();

        //shooter.SPINNER_SPEED_NEAR = -1280;

        shooter.setPID(pid);

        shooterThread = new Thread(shooter);
        thisTeleThread = new Thread(this);

        shooterThread.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        int id = getTargetTag();
        thisTeleThread.start();

        shooter.setPID(pid);

        waitForStart();

        shooter.spinUp(false, true);

        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .lineToYSplineHeading(posHeadings.obeliskPos, ang(posHeadings.obeliskAngle));
        runTrajectory(traj);

        pattern = readObelisk();

        intake.go();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .turn(ang(-52));
        runTrajectory(traj);

        align(getTargetTag());

        shootThree();

        intake.setSpeed(-1.0);

        intake.go();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(ang(25))
                .strafeToLinearHeading(new Vector2d(4,  -50 * getInvert()), ang(90));
        runTrajectory(traj);

        Thread.sleep(1000);

        shooter.spinUp(false, false);

        traj = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(10, posHeadings.obeliskPos * getInvert()), ang(50));
                //.lineToXConstantHeading(-30);
        runTrajectory(traj);

        intake.stop();

        shootThree();

        intake.setSpeed(-1);

        intake.go();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(ang(25))
                .strafeToLinearHeading(new Vector2d(-30,  -50 * getInvert()), ang(90));
        runTrajectory(traj);

        Thread.sleep(1000);

        shooter.spinUp(false, false);

        traj = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(10, posHeadings.obeliskPos * getInvert()), ang(50));
        runTrajectory(traj);

        intake.stop();

        shootThree();

        traj = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(initPose.position, initPose.heading);
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

    private void updateAutoThread(){
        // Output pattern
        BallColor[] loadedColors = shooter.getLoadedColors();
        if(pattern != null)
            telemetry.addData("Pattern", pattern.toString());
        else
            telemetry.addLine("Waiting to read pattern...");
       // telemetry.addData("Static or Read", readOrStatic);
        telemetry.addData("Static Loaded",  "%s %s %s", loadedColors[0], loadedColors[1], loadedColors[2]);
        telemetry.addData("Live Loaded",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
        telemetry.addData("Next fire index", currentIndex);
        //telemetry.addData("Fired", "%d %d %d", fired[0], fired[1], fired[2]);
        telemetry.addLine(getCurrentPoseString());
        telemetry.addLine("==== SHOOTER: =====");
        telemetry.addData("Speed", shooter.getVelocity());
        telemetry.addData("Calculated speed", fieldMap.getStateAtPose(drive.localizer.getPose()).speed);
        telemetry.addData("Shooter Thread is alive", shooterThread.isAlive());
        telemetry.addData("Shooter state", shooter.getState());
        //telemetry.addData("Current is shoot", shooter.commandStackEmpty());
        //telemetry.addLine(shooter.getCommandStackString());
        telemetry.update();

        // Use field data
        drive.localizer.update();
        //shooter.setFiringSpeed(fieldMap.getStateAtPose(drive.localizer.getPose()).speed);
        shooter.SPINNER_SPEED_NEAR = fieldMap.getStateAtPose(drive.localizer.getPose()).speed;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Vel", shooter.getVelocity());
        packet.put("Target Vel", -(shooter.SPINNER_SPEED_NEAR));
        packet.put("Left shooter Vel", shooter.getLeftVelocity());
        packet.put("Right shooter Vel", shooter.getRightVelocity());

        dashboard.sendTelemetryPacket(packet);

    }

    private void align(int tagId){
        AprilTagDetection tag = tBrain.getTagID(tagId);
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

    private void shootNext(boolean wait) {
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
                //shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.LEFT, true));
                break;
            case 1:
                shooter.setShootSpecific(false, true, false);
                //shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.MID, true));
                break;
            case 2:
                shooter.setShootSpecific(false, false, true);
                //shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.RIGHT, true));
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

        waitForShooter();
    }

    private void shootThree(){
        // Read
        shooter.updateLoadedColors();
        rotateToFire();
        for(int i = 0; i < 3; i++){
            shooter.spinUp(false, false);
            shootNext(true);
            // Poor hack
            //if(i == 0) shooter.SPINNER_SPEED_NEAR = -1300;
            //if(i > 0) shooter.SPINNER_SPEED_NEAR = -1350;

            //sleep(200);
            //while(!(Math.abs(shooter.SPINNER_SPEED_NEAR) - 10 < shooter.getVelocity() && Math.abs(shooter.SPINNER_SPEED_NEAR) + 10 > shooter.getVelocity())){}
            //shooter.pushCommand(new ShooterCommands.SpinUp(false, false));
        }
        // Reset
        fired = new boolean[3];
        shooter.stopShoot();
        shooter.kickersWait();
        //shooter.pushCommand(new ShooterCommands.StopCommand());
    }

    public String getCurrentPoseString() {
        Pose2d pose = drive.localizer.getPose();
        return String.format("(x=%.2f, y=%.2f, h=%.2f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
    }

    public void rotateToFire(){
        TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                .turnTo(fieldMap.getStateAtPose(drive.localizer.getPose()).heading + ang(10));
        Actions.runBlocking(traj.build());
    }

    protected abstract int getTargetTag();

    protected abstract int getInvert();
}
