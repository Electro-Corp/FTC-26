package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.fieldmodeling.DataLogger;
import org.firstinspires.ftc.teamcode.fieldmodeling.FieldDataPoints;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BallColor;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.io.FileReader;
import java.io.Reader;


public abstract class MainTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;


    private final boolean rrEnabled = false;

    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private ColorSensors colorSensors;
    private Intake intake;
    private Shooter shooter;

    public static PIDFCoefficients pid = new PIDFCoefficients(30,0.3,0.5,12.5);


    private boolean erroredOut = false;
    private String readError = "NONE";
    private boolean xPrev = false;
    private boolean xCurr = false;
    private boolean autoAim = true;

    private boolean shootThreeSpeed = false;

    public FieldDataPoints fieldMap;


    private void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotorEx.class,"leftFront");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");

//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Aiming
        tBrain = new TestBrain(hardwareMap);

        try{
            initPose = getAutoPose();
        } catch (Exception e){
            initPose = new Pose2d(50,-50, Math.toRadians(-50 * GetSideMultiplier()));
            erroredOut = true;
            readError = e.getMessage();
        }

        if(rrEnabled) {
            drive = new MecanumDrive(hardwareMap, initPose);
            drive.localizer.setPose(initPose);
        }
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter(hardwareMap, colorSensors, false);
        intake = new Intake(hardwareMap, shooter);

        shooter.setPID(pid);

        // Init field
        fieldMap = DataLogger.read();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        runtime.reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        while (opModeIsActive()){
            updateDriveMotors();

            telemetry.addData("READY TO FIRE?", (Math.abs(shooter.SPINNER_SPEED_NEAR) - 10 < shooter.getVelocity() && Math.abs(shooter.SPINNER_SPEED_NEAR) + 10 > shooter.getVelocity()));
            telemetry.addData("SHOOT THREE MODE", shootThreeSpeed);
            telemetry.addLine("=========================================");
            telemetry.addData("Shooter State", shooter.getState());
            telemetry.addData("LOADED",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            if(rrEnabled) {
                telemetry.addData("Estimated values", "%f | (%f, %f, %f)", shooter.SPINNER_SPEED_NEAR, fieldMap.getStateAtPose(drive.localizer.getPose()).posX, fieldMap.getStateAtPose(drive.localizer.getPose()).posY, Math.toDegrees(fieldMap.getStateAtPose(drive.localizer.getPose()).heading));
            }
            if(erroredOut){
                telemetry.addLine("Failed to read auto end position, using default.");
                telemetry.addLine(readError);
            }

            aimAssist();

//            telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size()); // How many are on the screen?
//            AprilTagDetection tag = tBrain.getTagID(GetSideMultiplier()); //
//            if (tag != null) {
//                telemetry.addData("Bearing to target", tag.ftcPose.bearing);
//                telemetry.addData("Bearing (rad) to target", Math.toRadians(tag.ftcPose.bearing));
//                telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
//            }

            if(rrEnabled)
                drive.localizer.update();


            telemetry.addLine(getCurrentPoseString());

            // Update speed with position
            if(rrEnabled)
                shooter.SPINNER_SPEED_NEAR = fieldMap.getStateAtPose(drive.localizer.getPose()).speed;
            else shooter.SPINNER_SPEED_NEAR = -1300;
            if(shootThreeSpeed) shooter.SPINNER_SPEED_NEAR -= 60;
            //else shooter.SPINNER_SPEED_NEAR -= 10;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter Vel", shooter.getVelocity());
            packet.put("Target Vel", -(shooter.SPINNER_SPEED_NEAR));
            packet.put("Left shooter Vel", shooter.getLeftVelocity());
            packet.put("Right shooter Vel", shooter.getRightVelocity());

            dashboard.sendTelemetryPacket(packet);

            readGamepad();
            telemetry.update();
        }
    }

    // Aiming Variables
    private boolean isYPressed = false;
    private double incAmount = 0.1;
    private void aimAssist(){
        if(gamepad1.y) {
            if (!isYPressed) {
                AprilTagDetection tag = tBrain.getTagID(GetSideMultiplier()); // Only Red tag right now
                if (tag != null) {
                    AprilTagPoseFtc tagPose = tag.ftcPose;
                    incAmount = Math.toRadians(tagPose.bearing) / 2;

                    Action trajAction = null;

                    TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
                        .turn(incAmount);
                    trajAction = trajectory.build();

                    Actions.runBlocking(trajAction);
                }
            }
            isYPressed = true;
        }
        else{
            isYPressed = false;
        }
    }

    private void updateDriveMotors() {
        double max = 0.0;
        double axial = 0;
        double lateral = 0;
        double yaw = 0;

        if (Math.abs(gamepad1.left_stick_y) > 0.1)
            axial = -gamepad1.left_stick_y;
        if (Math.abs(gamepad1.left_stick_x) > 0.1)
            lateral = gamepad1.left_stick_x;
        if (Math.abs(gamepad1.right_stick_x) > 0.1)
            yaw = gamepad1.right_stick_x;

        if (gamepad1.dpad_up)
            axial += 0.3;
        if (gamepad1.dpad_down)
            axial -= 0.3;
        if (gamepad1.dpad_left)
            lateral -= 0.3;
        if (gamepad1.dpad_right)
            lateral += 0.3;

        if (gamepad1.right_bumper)
            yaw -= 0.3 * -1;
        if (gamepad1.left_bumper) {
            yaw += 0.3 * -1;
        }

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if (gamepad1.a) {
            leftFrontPower *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower *= 0.5;
            rightBackPower *= 0.5;
        }

        telemetry.addData("LF", leftFrontPower);
        telemetry.addData("RF", rightFrontPower);
        telemetry.addData("LB", leftBackPower);
        telemetry.addData("RB", rightBackPower);


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    boolean shooting = false, gateHeld = false, aHeld = false;
    private void readGamepad(){
        if(gamepad1.y){
            rotateToFire();
        }

        //control intake, gamepad 2 left trigger is forward and left bumper is reverse
        if(gamepad2.left_trigger >= .2) {
            intake.setSpeed(-gamepad2.left_trigger);
            intake.go();
            //shooter.setDamUp();
        }
        else if(gamepad2.left_bumper) {
            intake.reverse();
        }
        else {
            intake.stop();
        }

        boolean fast = false;//gamepad2.b;
        shooting = shooter.isShooting();
        //shooter
        if(gamepad2.right_bumper /*&& !shooting*/) { //shoot far
            rotateToFire();
            //shooter.kickersWait();
            shooter.setToShootAll();
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.right_trigger > .2){
            shooter.spinUp(fast);
            //shooter.setDamDown();
        }
        if(gamepad2.b){
            shooter.reverseHumanPlayer(fast);
        }
        // Uncomment later
//        if(gamepad2.x){
//            rotateToFire();
//            if(fast)
//                shooter.shootColorFar(BallColor.PURPLE);
//            else
//                shooter.shootColorNear(BallColor.PURPLE);
//        }
        if(gamepad2.a){
            if(!aHeld) {
                shootThreeSpeed = !shootThreeSpeed;
                aHeld = true;
            }
        }else{
            aHeld = false;
        }
        // Manual shoot three
        if(gamepad2.dpad_left){
            rotateToFire();
            // Left
            shooter.setShootSpecific(true, false, false);
            // Ugly but
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.dpad_up){
            rotateToFire();
            // Center
            shooter.setShootSpecific(false, true, false);
            // Ugly but
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.dpad_right){
            rotateToFire();
            // Right
            shooter.setShootSpecific(false, false, true);
            // Ugly but
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.y){
            shooter.stopShoot();
        }

        xCurr = gamepad1.x;

        if (xCurr && !xPrev) {
            if (autoAim) autoAim = false;
            else autoAim = true;
        }
        xPrev = xCurr;

        shooter.update();
    }

    public void rotateToFire(){
        if (autoAim && rrEnabled) {
            Pose2d og = drive.localizer.getPose();
            Pose2d mapping = new Pose2d(new Vector2d(og.position.x, og.position.y * GetSideMultiplier()), og.heading.toDouble());
            TrajectoryActionBuilder traj = drive.actionBuilder(drive.localizer.getPose())
                    .turnTo(((fieldMap.getStateAtPose(mapping).heading * GetSideMultiplier())));
            Actions.runBlocking(traj.build());
        }
    }

    public String getCurrentPoseString() {
        if(rrEnabled) {
            Pose2d pose = drive.localizer.getPose();
            return String.format("(x=%.2f, y=%.2f, h=%.2f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        }else{
            return "none";
        }
    }

    public Pose2d getAutoPose() throws Exception {
        Reader reader = new FileReader("/sdcard/end.json");
        JsonObject obj = new JsonParser().parse(reader).getAsJsonObject();
        JsonObject wrapper = obj.get("start").getAsJsonObject();
        return new Pose2d(new Vector2d(wrapper.get("posX").getAsDouble(), wrapper.get("posY").getAsDouble()), wrapper.get("heading").getAsDouble());
    }

    protected abstract int GetSideMultiplier();

}
