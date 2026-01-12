package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private double speed;

    private Shooter shooter;
    public Intake (HardwareMap hardwareMap, Shooter shooter){
        speed = -1.0;
        this.shooter = shooter;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public Intake (HardwareMap hardwareMap){
        speed = -1.0;
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void setSpeed(double s){
        this.speed = s;
    }

    public void go(){
        intakeMotor.setPower(speed);
        if(this.shooter != null){
            if(this.shooter.getState() == Shooter.ShooterState.STOPPED){
                this.shooter.reverse(false);
            }
        }
    }

    public void reverse(){
        intakeMotor.setPower(-speed);
    }

    public void stop(){
        intakeMotor.setPower(0);
    }

    public Action goAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                go();
                return false;
            }
        };
    }

    /**
     * Returns immediately
     */
    public Action reverseAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeMotor.setPower(-speed);
                return false;
            }
        };
    }

    /**
     * Returns immediately
     */
    public Action stopAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeMotor.setPower(0);
                return false;
            }
        };
    }

}
