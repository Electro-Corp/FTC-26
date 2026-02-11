package org.firstinspires.ftc.teamcode.steelheads;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class SparkyNav
{
    GoBildaPinpointDriver odometry;
    ElapsedTime timer = new ElapsedTime();
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;

    public double teleOpIMUOffset;

    public int navCorrections = 0;

    double currentTarget = 0;



    public double navAcceleration = 0;

    public double distanceToTargetPrevious = 10000;
    public double distanceTraveledFromPrevious = 0.0;
    public SparkyNav(GoBildaPinpointDriver pod)
    {
        odometry = pod;
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    public SparkyNav()
    {

    }

    public void sleep(int milliseconds)
    {
        timer.reset();
        while(timer.milliseconds() < milliseconds)
        {
            //do nothing
        }
    }

    public void setMotors(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        motorFrontLeft = fl;
        motorFrontRight = fr;
        motorBackLeft = bl;
        motorBackRight = br;
    }

    public double[] navToPosition(double navXTarget, double navYTarget, double navOrientationTarget, boolean navOnlyRotate,
                                  boolean navPrecise, double navSpeedOverride, boolean navEngaged)
    {
        // variables that you can change to compensate for differences in motors
        double rotationErrorTolerance = 2.5; // error tolerance for rotation in degrees
        double distanceToRotationSlow = 40; // slow rotational speed on linear function if below this value
        double distanceToNavSlow = 200; // slow robot speed on linear function when distance to target lower than this value
        double navErrorTolerance = 25; // error tolerance for distance to target in mm
        double powerMaxRotate = 0.75; // max power used for rotation
        double powerMinRotate = 0.2; // min power used for rotation (increase if robot stops before reaching allowed error)
        double powerMaxNav = 1.0; // max power used for movement

        odometry.update();

        double distanceToTarget = 0.0;
        double distanceToTargetOrientation = 0.0;
        double currentOrientation = getOrientationCurrent();
        double currentOrientationRad = getOrientionRad();
        double currentX = getX();
        double currentY = getY();

        if(navSpeedOverride > 0)
        {
            // override the max power for movement for more precise movements or short distances
            powerMaxNav = navSpeedOverride;
        }
        double powerMinNav = 0.25; // min power for movement (increase if robot stops before reaching allowed error)

        double rotationX = 0;
        double rotationY = 0;

        double powerMotorNav = powerMaxNav;

        double bearingTarget = 0.0;
        double valueX = 0.0;
        double valueY = 0.0;
        double valueRotation = 0.0;
        double rotationDirection = 1.0;  // 1.0 is CW -1.0 is CCW

        if (navEngaged)
        {
            distanceToTargetOrientation = navOrientationTarget - currentOrientation;

            // converts the orientation to values between -180 and +180
            if(distanceToTargetOrientation < -180)
            {
                distanceToTargetOrientation += 360;
            }
            else if(distanceToTargetOrientation > 180)
            {
                distanceToTargetOrientation -= 360;
            }

            // determines which direction of rotation is closest to target
            if(distanceToTargetOrientation < 0)
            {
                rotationDirection = -1.0;
            }

            if (Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
            {
                if(navOnlyRotate)
                {
                    // stop rotation if still within error after short sleep
                    motorFrontLeft.setPower(0.0);
                    motorFrontRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    sleep(100);
                    odometry.update();
                    distanceToTargetOrientation = navOrientationTarget - getOrientationCurrent();

                    // converts the orientation to values between -180 and +180
                    if (distanceToTargetOrientation < -180)
                    {
                        distanceToTargetOrientation += 360;
                    }
                    else if (distanceToTargetOrientation > 180)
                    {
                        distanceToTargetOrientation -= 360;
                    }

                    if (distanceToTargetOrientation > 0)
                    {
                        rotationDirection = -1.0;
                    }
                }
                if (Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
                {
                    valueRotation = 0.0;
                    if(navOnlyRotate)
                    {
                        motorFrontLeft.setPower(0);
                        motorFrontRight.setPower(0);
                        motorBackLeft.setPower(0);
                        motorBackRight.setPower(0);
                        navEngaged = false;
                    }
                }
                else
                {
                    valueRotation = powerMinRotate * rotationDirection;
                }
            }
            else if (Math.abs(distanceToTargetOrientation) < 8.0)
            {
                // uses min rotation power when close to target
                valueRotation = powerMinRotate * rotationDirection;
            }
            else if (Math.abs(distanceToTargetOrientation) <= distanceToRotationSlow)
            {
                //
                if ((Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) < 5)
                        && Math.abs(distanceToTargetOrientation) > 10)
                {
                    powerMaxRotate = powerMaxRotate * 0.8;
                }
                else
                {
                    if(powerMaxRotate > 0.5)
                    {
                        powerMaxRotate = 0.5;
                    }
                }

                if (Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) > 120)
                {
                    // if the speed of rotation is too high when within the distanceToRotationSlow value, turn off rotation
                    valueRotation = 0;
                }
                else
                {
                    if (Math.abs(odometry.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * (180 / Math.PI)) < 5)
                    {
                        // if barely rotating and powerMinRotate less than 0.2 up to 0.2
                        if (powerMinRotate < 0.2)
                        {
                            powerMinRotate = 0.2;
                        }
                    }
                    // slow rotation using linear function based on distanceToTarget
                    valueRotation = (((powerMaxRotate - powerMinRotate) *
                            (Math.abs(distanceToTargetOrientation) / distanceToRotationSlow)) + powerMinRotate) * rotationDirection;
                }
            }
            else
            {
                valueRotation = powerMaxRotate * rotationDirection;
            }

            if(navOnlyRotate)
            {
                motorFrontLeft.setPower((valueRotation));
                motorFrontRight.setPower((valueRotation) * -1);
                motorBackLeft.setPower((valueRotation));
                motorBackRight.setPower((valueRotation) * -1);
            }
            else
            {
                // determine then bearing target (direction you would push stick to go towards target location)
                double radiansAdd = 0.0;
                if ((navXTarget - currentX) < 0 && (navYTarget - currentY) < 0)
                {
                    radiansAdd = Math.toRadians(180);
                }
                else if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) < 0)
                {
                    radiansAdd = Math.toRadians(180);
                }

                if ((navXTarget - currentX) >= 0 && (navYTarget - currentY) == 0)
                {
                    bearingTarget = Math.toRadians(90);
                }
                else if ((navXTarget - currentX) < 0 && (navYTarget - currentY) == 0)
                {
                    bearingTarget = Math.toRadians(270);
                }
                else
                {
                    bearingTarget = Math.atan((navXTarget - currentX) / (navYTarget - currentY)) + radiansAdd;
                }

                valueX = Math.sin(bearingTarget);
                valueY = Math.cos(bearingTarget);

                if (valueX >= valueY && valueY != 0)
                {
                    valueY = valueY / Math.abs(valueX);
                    valueX = valueX / Math.abs(valueX);
                }
                else if (valueY > valueX && valueX != 0)
                {
                    valueX = valueX / Math.abs(valueY);
                    valueY = valueY / Math.abs(valueY);
                }

                distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));

                if (distanceToTargetPrevious == 10000)
                {
                    // distanceToTargetPrevious is the last distance to the target
                    // used to determine how far you've gone
                    // setting distanceToTargetPrevious to 10000 when setting navEngaged to true resets this value
                    distanceToTargetPrevious = distanceToTarget;
                    navAcceleration = 0;
                }

                navAcceleration = navAcceleration + 0.1;
                if (navAcceleration < 1)
                {
                    // for the first 10 loops through the code, the power to wheels is slowly increased
                    // this stops the robot from jerking when starting
                    powerMaxNav = powerMaxNav * navAcceleration;
                }

                distanceTraveledFromPrevious = Math.abs(distanceToTarget - distanceToTargetPrevious);

                powerMotorNav = powerMaxNav;

                if (distanceToTarget < distanceToNavSlow)
                {
                    // slow the robot down as it gets closer to target unless navPrecise is false
                    if(navPrecise)
                    {
                        if(powerMaxNav > 0.5)
                        {
                            powerMaxNav = 0.5;
                        }

                        // slow down

                        if (distanceToTarget < 100)
                        {
                            // if within 100 mm from target, use min power
                            powerMotorNav = powerMinNav;
                        }
                        else
                        {
                            // use linear function to reduce power as robot gets closer to target
                            powerMotorNav = ((powerMaxNav - powerMinNav) * (distanceToTarget / distanceToNavSlow)) + powerMinNav;
                        }
                    }
                }

                if(!navPrecise && (distanceToTarget <= navErrorTolerance + 20))
                {
                    // when navPrecise if false just getting close turns off the power to the wheels
                    // robot will not slow down as it nears the target
                    // this can be used to set a waypoint to travel through before moving to another location
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    navEngaged = false;
                }
                else if (distanceToTarget > navErrorTolerance || Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                {
                    if(navPrecise)
                    {
                        if (distanceTraveledFromPrevious > 15 && distanceToTarget < distanceToNavSlow)
                        {
                            // if the robot is moving too fast in the slow does zone, tap the brakes
                            powerMotorNav = -0.20;
                        }

                        // navCorrections allows you to set a value for how many times the robot tries to correct the position
                        // each time it has to correct within the error + 10 mm it ups the navCorrection value
                        // this stops the robot from getting stuck in a correction loop and just eventually stops
                        // it does result in sometimes being outside the margin of error, but not by much
                        if (distanceToTarget < navErrorTolerance + 10)
                        {
                            navCorrections++;
                        }
                        else
                        {
                            navCorrections = 0;
                        }
                    }
                    rotationX = (valueX * Math.cos(currentOrientationRad) - valueY * Math.sin(currentOrientationRad));
                    rotationY = (valueX * Math.sin(currentOrientationRad) + valueY * Math.cos(currentOrientationRad));

                    if(distanceToTarget <= navErrorTolerance && Math.abs(distanceToTargetOrientation) > rotationErrorTolerance)
                    {
                        // just rotate if within the margin of error for target position
                        motorFrontLeft.setPower((valueRotation));
                        motorFrontRight.setPower((valueRotation) * -1);
                        motorBackLeft.setPower((valueRotation));
                        motorBackRight.setPower((valueRotation) * -1);
                    }
                    else
                    {
                        // passes the calculated values to the field-centric driving code
                        double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(valueRotation), 1);
                        motorFrontLeft.setPower(((rotationY + rotationX + valueRotation) / denominator) * powerMotorNav);
                        motorFrontRight.setPower(((rotationY - rotationX - valueRotation) / denominator) * powerMotorNav);
                        motorBackLeft.setPower(((rotationY - rotationX + valueRotation) / denominator) * powerMotorNav);
                        motorBackRight.setPower(((rotationY + rotationX - valueRotation) / denominator) * powerMotorNav);
                    }
                }
                else
                {
                    // sometimes the robot is within the margin of error, but momentum moves it out
                    // this code stops the robot and waits a moment and then tests again before ending navigation
                    motorFrontLeft.setPower(0);
                    motorFrontRight.setPower(0);
                    motorBackLeft.setPower(0);
                    motorBackRight.setPower(0);
                    sleep(100);

                    odometry.update();
                    currentOrientation = getOrientationCurrent();
                    currentX = getX();
                    currentY = getY();

                    distanceToTarget = Math.abs(Math.sqrt((Math.pow((navXTarget - currentX), 2)) + (Math.pow((navYTarget - currentY), 2))));
                    distanceToTargetOrientation = navOrientationTarget - currentOrientation;
                    if (distanceToTargetOrientation < -180)
                    {
                        distanceToTargetOrientation = distanceToTargetOrientation + 360;
                    }
                    else if (distanceToTargetOrientation > 180)
                    {
                        distanceToTargetOrientation = distanceToTargetOrientation - 360;
                    }

                    if ((distanceToTarget <= navErrorTolerance && Math.abs(distanceToTargetOrientation) <= rotationErrorTolerance)
                            || navCorrections >= 3)
                    {

                        navEngaged = false;
                        navCorrections = 0;
                    }
                }
            }
        }

        // set the previous value to the current value so next pass through you can get the change
        distanceToTargetPrevious = distanceToTarget;

        double[] navReturn = new double[3];
        if(navEngaged)
        {
            // navigation is still engaged
            navReturn[0] = 1;
        }
        else
        {
            // navigation is disengaged
            navReturn[0] = -1;
        }

        // values below were added so we could add them to the telemetry
        navReturn[1] = distanceToTarget;
        navReturn[2] = distanceToTargetOrientation;
        return navReturn;
    }

    //misc PinPoint functions

    public void reset()
    {
        odometry.resetPosAndIMU();
        teleOpIMUOffset = 0;
    }

    public void recalibrate()
    {
        odometry.recalibrateIMU();
    }

    public double getOrientationRaw()
    {
        return (odometry.getHeading(UnnormalizedAngleUnit.RADIANS) * (180/Math.PI));
    }

    public double getOrientationCurrent()
    {
        return (getOrientationRaw() + 360000) % 360;
    }

    public void setDirection(GoBildaPinpointDriver.EncoderDirection xPod, GoBildaPinpointDriver.EncoderDirection yPod)
    {
        odometry.setEncoderDirections(xPod, yPod);
    }

    public double getOrientionRad()
    {
        return odometry.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    public void update()
    {
        odometry.update();
    }

    public double getX()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = 1;
        return odometry.getPosX(DistanceUnit.MM) * reverseDirection;
    }

    public double getY()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = -1;
        return odometry.getPosY(DistanceUnit.MM) * reverseDirection;
    }

    public double getVelX()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = 1;
        return odometry.getVelX(DistanceUnit.MM) * reverseDirection;
    }

    public double getVelY()
    {
        // if pod is reversed multiply by -1
        double reverseDirection = -1;
        return odometry.getVelY(DistanceUnit.MM) * reverseDirection;
    }

    public double getTargetPos()
    {
        return currentTarget;
    }

}
