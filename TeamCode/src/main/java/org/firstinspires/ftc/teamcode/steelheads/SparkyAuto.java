package org.firstinspires.ftc.teamcode.steelheads;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;

@Autonomous
public class SparkyAuto extends OpMode
{
    double navXTarget = 0;
    double navYTarget = 0;
    double navOrientationTarget = 0;
    boolean navOnlyRotate = false;
    boolean navPrecise = false;
    double navSpeedOverride = 0;
    boolean navEngaged = false;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    ElapsedTime timer;
    String step;
    GoBildaPinpointDriver odometry;
    double distanceToTarget = 0;
    double distanceToTargetOrientation = 0;

    SparkyNav nav;


    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }

    public void start()
    {
        step = "start";
    }

    public void init()
    {
        timer = new ElapsedTime();

        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");

        // VERY IMPORTANT - offsets need to be correct to work
        // we switched the X and Y odometry when plugging in to the odometry computer
        // goBilda has it sets X as forward and back and Y as side to side
        // we preferred Y as forward and back and X as side to side
        odometry.setOffsets(170, 75, DistanceUnit.MM);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();

        nav = new SparkyNav(odometry);
        nav.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

    }

    public void init_loop()
    {
        showTelemetry();
    }

    public void showTelemetry()
    {
        telemetry.addData("navXTarget", navXTarget);
        telemetry.addData("navYTarget", navYTarget);
        telemetry.addData("navOrientationTarget", navOrientationTarget);
        telemetry.addData("navOnlyRotate", navOnlyRotate);
        telemetry.addData("navPrecise", navPrecise);
        telemetry.addData("navEngaged", navEngaged);

        telemetry.addData("X Current", nav.getX());
        telemetry.addData("Y Current", nav.getY());
        telemetry.addData("Orientation Current", nav.getOrientationCurrent());

        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("distanceToTargetOrientation", distanceToTargetOrientation);

        telemetry.update();
    }

    public void Autonomous()
    {
        if(step.equalsIgnoreCase("start"))
        {
            // waits 500 ms before starting
            timer.reset();
            step = "movement test 1";
        }
        else if(step.equalsIgnoreCase("movement test 1") && timer.milliseconds() > 500)
        {
            //  moves forward 1000 mm
            navSpeedOverride = 0.9; // runs at set speed instead of powerMaxNav (powerMaxNav defined in SparkyNav)
            navXTarget = 0;  // target X coordinate
            navYTarget = 1000;  // target Y coordinate
            nav.distanceToTargetPrevious = 10000; // reset previous distance to target
            navOrientationTarget = 0; // desired ending orientation (0 - 360)
            navOnlyRotate = false; // if true ignores X and Y targets and only rotates
            navPrecise = true; // true will stop at this point, false will end navigation but not try to stop precisely at target
            navEngaged = true; // starts navigation
            step = "rotate test 1"; // set next step to run
        }
        else if(step.equalsIgnoreCase("rotate test 1") && !navEngaged)
        {
            // this step is triggered by navigation being disengaged
            // bot will rotate to 90 degrees
            navSpeedOverride = -1.0; // turns off navigation speed override and runs at max speed
            navOrientationTarget = 90;
            navOnlyRotate = true; // ignores target X and Y and only rotates
            navPrecise = true;
            navEngaged = true;
            step = "do something while rotating";
        }
        else if(step.equalsIgnoreCase("do something while rotating"))
        {
            // since we didn't specify the !navEngaged in the above if, this step will start running concurrently
            // DO SOMETHING
            step = "move and rotate";
        }
        else if(step.equalsIgnoreCase("move and rotate") && !navEngaged)
        {
            //  moves to back 500 and left 500
            //navSpeedOverride = -1.0; // no need to specify again since still defined in previous step
            navXTarget = -500;  // target X coordinate
            navYTarget = 500;  // target Y coordinate
            nav.distanceToTargetPrevious = 10000; // reset previous distance to target
            navOrientationTarget = 135; // desired ending orientation (0 - 360)
            navOnlyRotate = false; // if true ignores X and Y targets and only rotates
            navPrecise = true; // true will stop at this point, false will end navigation but not try to stop precisely at target
            navEngaged = true; // starts navigation
            step = "wait for a bit";
        }
        else if(step.equalsIgnoreCase("wait for a bit")  && !navEngaged)
        {
            timer.reset();
            step = "non-precise move";
        }
        else if(step.equalsIgnoreCase("non-precise move") && timer.milliseconds() > 500)
        {
            //  moves right from -500 to +500 and rotates to 0
            navXTarget = 500;  // target X coordinate
            navYTarget = 500;  // target Y coordinate
            nav.distanceToTargetPrevious = 10000; // reset previous distance to target
            navOrientationTarget = 0; // desired ending orientation (0 - 360)
            navOnlyRotate = false; // if true ignores X and Y targets and only rotates
            navPrecise = false; // will end navigation but not try to stop precisely at target
            navEngaged = true; // starts navigation
            step = "head back home";
        }
        else if(step.equalsIgnoreCase("head back home") && !navEngaged)
        {
            // should start going toward home after moving straight sideway but not coming to a stop
            navXTarget = 0;  // target X coordinate
            navYTarget = 50;  // target Y coordinate (not quite home, but shouldn't hit wall)
            nav.distanceToTargetPrevious = 10000; // reset previous distance to target
            navOrientationTarget = 0; // desired ending orientation (0 - 360)
            navOnlyRotate = false; // if true ignores X and Y targets and only rotates
            navPrecise = true; // will end navigation but not try to stop precisely at target
            navEngaged = true; // starts navigation
            step = "end";
        }
    }

    public void loop()
    {

        Autonomous(); // runs your autonomous code steps

        if(navEngaged)
        {
            // runs navigation code if navEngaged is true
            double[] navReturn = new double[3];
            navReturn = nav.navToPosition(navXTarget, navYTarget, navOrientationTarget, navOnlyRotate, navPrecise, navSpeedOverride, navEngaged);
            if (navReturn[0] == -1)
            {
                // returns +1 if still engaged and -1 if navigation has ended
                navEngaged = false;
            }
            distanceToTarget = navReturn[1]; // used to display distance to target in telemetry
            distanceToTargetOrientation = navReturn[2]; // used to display orientation distance to target in telemetry
        }
        showTelemetry();
    }
}
