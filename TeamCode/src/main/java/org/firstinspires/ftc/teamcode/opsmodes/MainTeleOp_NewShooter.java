package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
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
@TeleOp(name="New Shooter TeleOp")
public class MainTeleOp_NewShooter extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private ColorSensors colorSensors;
    private Intake intake;
    private Shooter_New shooter;

    public static PIDFCoefficients pid = new PIDFCoefficients(12,0.3,1,12);


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
        initPose = new Pose2d(0,0,0);
        drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new Shooter_New(hardwareMap, colorSensors);

        //shooter.setPID(pid);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            updateDriveMotors();

            //telemetry.addData("Shooter State", shooter.getState());
            telemetry.addData("LOADED",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            aimAssist();

//            telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size()); // How many are on the screen?
//            AprilTagDetection tag = tBrain.getTagID(GetMyTag()); //
//            if (tag != null) {
//                telemetry.addData("Bearing to target", tag.ftcPose.bearing);
//                telemetry.addData("Bearing (rad) to target", Math.toRadians(tag.ftcPose.bearing));
//                telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
//            }

            readGamepad();
            telemetry.update();
        }
    }

    // Aiming Variables
    private boolean isYPressed = false;
    private double incAmount = 0.1;
    private void aimAssist(){
//        if(gamepad1.y) {
//            if (!isYPressed) {
//                AprilTagDetection tag = tBrain.getTagID(GetMyTag()); // Only Red tag right now
//                if (tag != null) {
//                    AprilTagPoseFtc tagPose = tag.ftcPose;
//                    incAmount = Math.toRadians(tagPose.bearing) / 2;
//
//                    Action trajAction = null;
//
//                    TrajectoryActionBuilder trajectory = drive.actionBuilder(drive.localizer.getPose())
//                        .turn(incAmount);
//                    trajAction = trajectory.build();
//
//                    Actions.runBlocking(trajAction);
//                }
//            }
//            isYPressed = true;
//        }
//        else{
//            isYPressed = false;
//        }
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
        //shooting = shooter.isShooting();
        //shooter
        if(gamepad2.right_bumper) {

        }
        // Spin up
        if(gamepad2.right_trigger > .2){
            shooter.pushCommand(new ShooterCommands.SpinUp(false, false));
        }
        // Reverse
        if(gamepad2.b){
            shooter.pushCommand(new ShooterCommands.SpinUp(false, true));
        }
        // Color Shooting
        if(gamepad2.x){
            shooter.pushCommand(new ShooterCommands.ShootColorCommand(BallColor.PURPLE));
        }
        if(gamepad2.a){
            shooter.pushCommand(new ShooterCommands.ShootColorCommand(BallColor.GREEN));
        }
        // Manual shoot three
        if(gamepad2.dpad_left){
            shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.LEFT));
        }
        if(gamepad2.dpad_up){
            shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.MID));
        }
        if(gamepad2.dpad_right){
            shooter.pushCommand(new ShooterCommands.ShootCommand(Kickers.Position.RIGHT));
        }
        // Stop
        if(gamepad2.y){
            shooter.pushCommand(new ShooterCommands.StopCommand());
        }
        shooter.update();
    }

}
