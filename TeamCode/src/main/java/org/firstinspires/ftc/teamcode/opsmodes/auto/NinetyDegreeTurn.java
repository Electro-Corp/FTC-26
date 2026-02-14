package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.steelheads.SparkyNav;
@Autonomous
public class NinetyDegreeTurn extends OpMode{
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    SparkyNav nav;
    GoBildaPinpointDriver odometry;
    ElapsedTime timer;

    // NAV VARS
    double navXTarget = 0;
    double navYTarget = 0;
    double navOrientationTarget = 0;
    boolean navOnlyRotate = false;
    boolean navPrecise = false;
    double navSpeedOverride = 0;
    boolean navEngaged = false;
    double distanceToTarget = 0;
    double distanceToTargetOrientation = 0;


    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBack");


        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odometry.setOffsets(0, 0, DistanceUnit.MM);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setYawScalar(-1.0);
        odometry.resetPosAndIMU();

        nav = new SparkyNav(odometry);
        nav.setMotors(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
    }


    int steps = 0;
    boolean first = true;

    @Override
    public void loop() {

        if(!navEngaged){
            switch(steps){
            case 0:
                navSpeedOverride = -1.0;
                navOrientationTarget = 360;
                navOnlyRotate = true;
                navEngaged = true;
                break;
            case 1:
                navXTarget = 0;  // target X coordinate
                navYTarget = 500;  // target Y coordinate
                nav.distanceToTargetPrevious = 10000; // reset previous distance to target
                navOrientationTarget = 0; // desired ending orientation (0 - 360)
                navOnlyRotate = false; // if true ignores X and Y targets and only rotates
                navPrecise = true; // true will stop at this point, false will end navigation but not try to stop precisely at target
                navEngaged = true; // starts navigation
                break;
            }
        }

        if(navEngaged)
        {
            // runs navigation code if navEngaged is true
            double[] navReturn = new double[3];
            navReturn = nav.navToPosition(navXTarget, navYTarget, navOrientationTarget, navOnlyRotate, navPrecise, navSpeedOverride, navEngaged);
            if (navReturn[0] == -1)
            {
                steps++;
                // returns +1 if still engaged and -1 if navigation has ended
                navEngaged = false;
            }
            distanceToTarget = navReturn[1]; // used to display distance to target in telemetry
            distanceToTargetOrientation = navReturn[2]; // used to display orientation distance to target in telemetry
        }

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

    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }
}
