package org.firstinspires.ftc.teamcode.sam;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DriveActions {

    private final MecanumDrive drive;

    // TODO: replace these with your real field coordinates
    // The origin is the exact center of the field
    // +x mean towards alliance side
    // -x means away from alliance side
    // +y means towards audience (away from obelisk)
    // -y means towards obelisk (away from the audience)
    private static final Pose2d INIT_POSE           = new Pose2d(0.0, 0.0, Math.toRadians(135));

    private static final Pose2d READ_OBELISK_POSE   = new Pose2d(10.0, 40.0, Math.toRadians(90));
    private static final Pose2d BASKET_POSE         = new Pose2d(20.0, 60.0, Math.toRadians(180));
    private static final Pose2d ROW_OF_BALLS1_POSE  = new Pose2d(30.0, 40.0, Math.toRadians(0));
    private static final Pose2d LAUNCH_POSE         = new Pose2d(50.0, 40.0, Math.toRadians(0));

    public DriveActions(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap, INIT_POSE);
    }

    /**
     * Moves from the current pose to the "read obelisk" pose.
     */
    public Action moveToReadObelisk() {
        return drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(
                        READ_OBELISK_POSE.position.x,
                        READ_OBELISK_POSE.position.y),
                        READ_OBELISK_POSE.heading)
                .build();
    }

    /**
     * Rotates in place so the robot faces the basket.
     */
    public Action rotateToBasket() {
        return drive.actionBuilder(drive.localizer.getPose())
                .turnTo(BASKET_POSE.heading)
                .build();
    }

    /**
     * Moves from the current pose to the first row of balls.
     */
    public Action moveToRowOfBalls1() {
        return drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(
                        ROW_OF_BALLS1_POSE.position.x,
                        ROW_OF_BALLS1_POSE.position.y),
                        ROW_OF_BALLS1_POSE.heading)
                .build();
    }

    /**
     * Moves from the current pose to the launch location.
     */
    public Action moveToLaunchLocation() {
        return drive.actionBuilder(drive.localizer.getPose())
                .strafeTo(new Vector2d(
                        LAUNCH_POSE.position.x,
                        LAUNCH_POSE.position.y))
                .turnTo(LAUNCH_POSE.heading)
                .build();
    }
}
