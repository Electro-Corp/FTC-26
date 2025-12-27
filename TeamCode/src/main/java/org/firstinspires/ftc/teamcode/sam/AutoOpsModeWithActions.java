Canpackage org.firstinspires.ftc.teamcode.sam;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.camera.TestBrain;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name="Auto with Actions")
public class AutoOpsModeWithActions extends LinearOpMode {
    TestBrain tBrain = null;
    Pose2d initPose = null;

    private Intake intake;
    private AutoShooter shooter;
    private ColorSensors colorSensors;


    private DriveActions driveActions;

    private void initHardware() {
        tBrain = new TestBrain(hardwareMap);
        driveActions = new DriveActions(hardwareMap);
        initPose = new Pose2d(0,0,0);
        intake = new Intake(hardwareMap);
        colorSensors = new ColorSensors(hardwareMap);
        shooter = new AutoShooter(hardwareMap, colorSensors, tBrain);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToReadObelisk(),
                shooter.readObelisk(),
                shooter.spinUp(false)));

        //Launch initial balls
        Actions.runBlocking(new SequentialAction(        
                driveActions.moveToLaunchLocation(),
                shooter.readBallColors(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                intake.goAction()));

        //Gather row 1 balls
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToRowOfBalls1()));

        //Fire row 1 balls
        Actions.runBlocking(new SequentialAction(
                driveActions.moveToLaunchLocation(),
                shooter.readBallColors(),
                shooter.fireNextBall(),
                shooter.fireNextBall(),
                shooter.fireNextBall()
        ));
    }
}
