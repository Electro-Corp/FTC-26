package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// See Web Browser Panels here: http://192.168.43.1:8001/
public class Constants {

    /*
     * PIDF TUNING GUIDE
     * =================
     * All three PIDF sets below (translational, heading, drive) share the same four terms.
     * Order of arguments: (P, I, D, F)
     *
     * P — Proportional
     *   Applies a correction force proportional to the current error (how far off we are right now).
     *   Increase: faster response, reaches target sooner. Too high → robot overshoots and oscillates.
     *   Decrease: slower, sluggish response. Too low → robot barely moves toward the target.
     *
     * I — Integral
     *   Accumulates error over time and corrects for persistent steady-state offsets.
     *   Increase: eliminates lingering small errors that P alone can't overcome. Too high → slow
     *             oscillation that builds over time ("integral windup").
     *   Decrease: tolerates a small residual error. For most FTC use, keep this very small or 0.
     *
     * D — Derivative
     *   Reacts to how fast the error is changing, acting as a brake as the robot approaches target.
     *   Increase: more damping, reduces overshoot and oscillation. Too high → sluggish, "sticky"
     *             motion that creeps toward the target very slowly.
     *   Decrease: less damping, snappier but more prone to oscillation. Too low → robot bounces
     *             past the target repeatedly.
     *
     * F — Feedforward
     *   A constant output added regardless of error, used to overcome static friction or gravity.
     *   Increase: helps the robot start moving immediately without waiting for error to build.
     *             Too high → robot drifts or overshoots even when already at target.
     *   Decrease: robot relies entirely on P/I/D. Usually fine unless there is a strong static
     *             friction or a consistent load to overcome.
     *
     * TURNHEADINGERRORTHRESHOLD
     * =========================
     * The robot's turn is considered "done" (isTurning() returns false) only when the heading
     * error is smaller than this value (in radians). Currently set to 3 degrees.
     *
     * Too low (e.g. < 1°): isTurning() may never return false if the PID oscillates slightly
     *                       around the target — the turn hangs indefinitely.
     * Too high (e.g. > 5°): isTurning() returns false while the robot is still noticeably off
     *                        heading, causing inaccurate shots or misaligned paths.
     */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.88)
            .forwardZeroPowerAcceleration(-45.07)
            .lateralZeroPowerAcceleration(-78.5)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.0, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0.01, 0.012, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.015,0.0,0.0,0.1,0.4))
            .turnHeadingErrorThreshold(Math.toRadians(3)); // see TURNHEADINGERRORTHRESHOLD note above

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.5)
            .strafePodX(6.75)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .xVelocity(65.65)
            .yVelocity(50.38)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            3,
            3);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FixedFollower(
                followerConstants,
                new PinpointLocalizer(hardwareMap, localizerConstants),
                new Mecanum(hardwareMap, driveConstants),
                pathConstraints);
    }
}