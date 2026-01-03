package org.firstinspires.ftc.teamcode.sam;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DriveActions {

    private final MecanumDrive drive;

    // The origin is the exact center of the field
    //         +x
    //          |
    //          |
    // +y  -----+----- -y
    //          |
    //          |
    //         -x
    //
    // -x means away towards the audience
    // +x mean towards the obelisk
    // -y means towards alliance
    // +y means away from alliance

//    private static final Pose2d INIT_POSE          = new Pose2d(50.0, -50.0, Math.toRadians(-45));
//    private static final Pose2d READ_OBELISK_POSE  = new Pose2d(10.0, -10.0, Math.toRadians(0));
//    private static final Pose2d ROW_OF_BALLS1_POSE = new Pose2d(13.0, -56.0, Math.toRadians(90));
//    private static final Pose2d LAUNCH_POSE        = new Pose2d(10.0, -10.0, Math.toRadians(-45));

    private static final Pose2d INIT_POSE          = new Pose2d(50.0, -50.0, Math.toRadians(-45));
    private static final Pose2d LAUNCH_POSE        = new Pose2d(0.0, 0.0, Math.toRadians(-45));
    private static final Pose2d READ_OBELISK_POSE  = new Pose2d(10.0, -10.0, Math.toRadians(0));
    private static final Pose2d ROW_OF_BALLS1_POSE = new Pose2d(0.0, 10.0, Math.toRadians(90));

    public DriveActions(HardwareMap hardwareMap) {
        this.drive = new MecanumDrive(hardwareMap, INIT_POSE);
    }

    public String getCurrentPoseString() {
        Pose2d pose = drive.localizer.getPose();
        return String.format("(x=%.2f, y=%.2f, h=%.2f)", pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    /**
     * Moves from the current pose to the "read obelisk" pose.
     */
    public Action moveToReadObelisk() {
        return drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(
                        READ_OBELISK_POSE.position.x,
                        READ_OBELISK_POSE.position.y),
                        READ_OBELISK_POSE.heading)
                .build();
    }

    /**
     * Moves from the current pose to the first row of balls.
     */
    public Action moveToRowOfBalls1() {
        return drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(
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
                .strafeToLinearHeading(new Vector2d(
                        LAUNCH_POSE.position.x,
                        LAUNCH_POSE.position.y),
                        LAUNCH_POSE.heading)
                .build();
    }
}
