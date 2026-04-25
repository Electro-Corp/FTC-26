package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;

/**
 * Fixes a bug in Pedro Pathing 2.0.6's Follower#turnTo where getSmallestAngleDifference
 * is unsigned, causing turnTo to always add to the current heading and turn the wrong
 * way when the target heading is less than the current heading.
 */
public class FixedFollower extends Follower {
    public FixedFollower(FollowerConstants constants, Localizer localizer,
                         Drivetrain drivetrain, PathConstraints pathConstraints) {
        super(constants, localizer, drivetrain, pathConstraints);
    }

    @Override
    public void turnTo(double radians) {
        double delta = radians - getHeading();
        while (delta > Math.PI)  delta -= 2 * Math.PI;
        while (delta < -Math.PI) delta += 2 * Math.PI;
        turn(Math.abs(delta), delta >= 0);
    }
}
