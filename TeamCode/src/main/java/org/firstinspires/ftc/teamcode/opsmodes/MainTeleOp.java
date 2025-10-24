package org.firstinspires.ftc.teamcode.opsmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;


public abstract class MainTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;

    TestBrain tBrain = null;

    Pose2d initPose = null;
    MecanumDrive drive = null;

    private Intake intake;
    private Shooter shooter;


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
        //tBrain = new TestBrain(hardwareMap);
        //initPose = new Pose2d(0,0,0);
        //drive = new MecanumDrive(hardwareMap, initPose);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            updateDriveMotors();
            //aimAssist();

            /*telemetry.addData("Total Tags on screen", tBrain.getVisibleTags().size()); // How many are on the screen?
            AprilTagDetection tag = tBrain.getTagID(GetMyTag()); // Only Red tag right now
            if (tag != null) {
                telemetry.addData("Bearing to target", tag.ftcPose.bearing);
                telemetry.addData("Bearing (rad) to target", Math.toRadians(tag.ftcPose.bearing));
                telemetry.addData("X Y Z", "| %.2f | %.2f | %.2f |", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z);
            }*/

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
                AprilTagDetection tag = tBrain.getTagID(GetMyTag()); // Only Red tag right now
                if (tag != null) {
                    AprilTagPoseFtc tagPose = tag.ftcPose;
                    incAmount = Math.toRadians(tagPose.bearing);

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

        telemetry.addData("LF", leftFrontPower);
        telemetry.addData("RF", rightFrontPower);
        telemetry.addData("LB", leftBackPower);
        telemetry.addData("RB", rightBackPower);


        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    boolean shooting = false;
    private void readGamepad(){
        //control intake, gamepad 2 left trigger is forward and left bumper is reverse
        intake.setSpeed(-gamepad1.left_trigger);
        if(gamepad1.left_trigger >= .2) {
            intake.go();
        }
        else if(gamepad1.y) {
            intake.reverse();
        }
        else {
            intake.stop();
        }

        shooting = shooter.isShooting();
        //shooter
        if(gamepad1.right_trigger >= .5 && !shooting) { //shoot far
            shooter.shootFar();
        }
        if(gamepad1.b && !shooting) { //shoot close
            shooter.shootNear();
        }
        shooter.update();
    }

    protected abstract int GetMyTag();

}