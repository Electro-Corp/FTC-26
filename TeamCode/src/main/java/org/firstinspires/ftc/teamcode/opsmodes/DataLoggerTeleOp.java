package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.fieldmodeling.DataLogger;
import org.firstinspires.ftc.teamcode.fieldmodeling.DataPoint;
import org.firstinspires.ftc.teamcode.fieldmodeling.FieldDataPoints;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name="LoggingTeleOp")
public class DataLoggerTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private ColorSensors colorSensors;
    private Intake intake;
    private Shooter shooter;

    private DataLogger dataLogger;
    private FieldDataPoints fieldDataPoints;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static PIDFCoefficients pid = new PIDFCoefficients(12,0.35,1,11);


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

        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter(hardwareMap, colorSensors, false);


        // Start logging
        dataLogger = new DataLogger();
        fieldDataPoints = new FieldDataPoints();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            updateDriveMotors();

            telemetry.addData("Shooter State", shooter.getState());
            telemetry.addData("LOADED",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addLine("================ LOGGING =================");
            telemetry.addData("Target Velocity", shooter.SPINNER_SPEED_NEAR);
            telemetry.addData("Localizer", "((%f %f), %f)", drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble());
            telemetry.addData("Logged points", fieldDataPoints.getNumOfPoints());

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter Vel", shooter.getVelocity());
            packet.put("Target Vel", -(shooter.SPINNER_SPEED_NEAR));
            packet.put("Left shooter Vel", shooter.getLeftVelocity());
            packet.put("Right shooter Vel", shooter.getRightVelocity());

            dashboard.sendTelemetryPacket(packet);

            drive.updatePoseEstimate();

            shooter.setPID(pid);

            readGamepad();
            telemetry.update();
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

    boolean shooting = false, gateHeld = false;
    private void readGamepad(){
        //control intake, gamepad 2 left trigger is forward and left bumper is reverse
        if(gamepad2.left_trigger >= .2) {
            intake.setSpeed(-gamepad2.left_trigger);
            intake.go();
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
            shooter.kickersWait();
            //shooter.setToShootAll();
            //if(fast)
            //    shooter.shootFar();
            //else shooter.shootNear();
        }
        if(gamepad2.right_trigger > .2){
            shooter.spinUp(fast);
        }
        if(gamepad2.b){
            shooter.reverse(fast);
        }
        // Uncomment later
        if(gamepad2.x){
            shooter.SPINNER_SPEED_NEAR += 1;
//            if(fast)
//                shooter.shootColorFar(BallColor.PURPLE);
//            else
//                shooter.shootColorNear(BallColor.PURPLE);
        }
        if(gamepad2.a){
            shooter.SPINNER_SPEED_NEAR -= 1;
//            if(fast)
//                shooter.shootColorFar(BallColor.GREEN);
//            else
//                shooter.shootColorNear(BallColor.GREEN);
        }
        // Manual shoot three
        if(gamepad2.dpad_left){
            // Left
            shooter.setShootSpecific(true, false, false);
            // Ugly but
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.dpad_up){
            // Center
            shooter.setShootSpecific(false, true, false);
            // Ugly but
            if(fast)
                shooter.shootFar();
            else shooter.shootNear();
        }
        if(gamepad2.dpad_right){
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
        shooter.update();

        if(gamepad1.startWasPressed()){
            updateLog();
        }


    }

    private void updateLog(){
        DataPoint packet = new DataPoint(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble(), shooter.SPINNER_SPEED_NEAR);

        fieldDataPoints.addDataPoint(packet);

        dataLogger.write(fieldDataPoints);
    }

}
