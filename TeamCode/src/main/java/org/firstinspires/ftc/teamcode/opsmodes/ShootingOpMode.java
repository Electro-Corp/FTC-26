package org.firstinspires.ftc.teamcode.opsmodes;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp(name="Shooting PID tune")
public class ShootingOpMode extends LinearOpMode {
    public static PIDFCoefficients pid = new PIDFCoefficients(30,0.3,0.5,12);

    double p, i , d, f = 0;

    public static int speed = 1300;


    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensors colorSensors = new ColorSensors(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, colorSensors, false);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        shooter.spinUp(false, true);

        while(!isStopRequested()){
            telemetry.addData("Shooter State", shooter.getState());
            telemetry.addData("LOADED",  "%s %s %s", colorSensors.readLeftColor(), colorSensors.readMidColor(), colorSensors.readRightColor());
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addData("L/R Vel", "%s, %s", shooter.getLeftVelocity(), shooter.getRightVelocity());
            telemetry.addData("Delta", Math.abs(shooter.getLeftVelocity() - shooter.getRightVelocity()));
            telemetry.update();

            shooter.SPINNER_SPEED_NEAR = -speed;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter Vel", shooter.getVelocity());
            packet.put("Target Vel", -(shooter.SPINNER_SPEED_NEAR));
            packet.put("Left shooter Vel", shooter.getLeftVelocity());
            packet.put("Right shooter Vel", shooter.getRightVelocity());

            dashboard.sendTelemetryPacket(packet);

            shooter.setPID(pid);

            boolean fast = false;

            if(gamepad2.right_trigger > .2){
                shooter.spinUp(fast);
            }

            if(gamepad2.y){
                shooter.stopShoot();
            }

            if(gamepad2.dpad_left){
                // Left
                shooter.setShootSpecific(true, false, false);
                // Ugly but
                if(fast)
                    shooter.shootFar();
                else shooter.shootNear();
            }
            if(gamepad2.dpad_up){
                // Center
                shooter.setShootSpecific(false, true, false);
                // Ugly but
                if(fast)
                    shooter.shootFar();
                else shooter.shootNear();
            }
            if(gamepad2.dpad_right){
                // Right
                shooter.setShootSpecific(false, false, true);
                // Ugly but
                if(fast)
                    shooter.shootFar();
                else shooter.shootNear();
            }
        }

    }
}
