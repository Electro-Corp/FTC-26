package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;

public class Shooter_New {
    public class Config {
        private static final long SPIN_UP_TIME_MS = 1800;
        private static final long SPIN_AFTER_SHOOT_MS = 200;
        private static final long PAUSE_UNTIL_GATE_OPEN = 400;
        private static final long PULL_KICKER_BACK = 750;
        public static final double SPINNER_SPEED_NEAR = -1300;
        public static final double SPINNER_SPEED_FAR = -7000;
    }

    private double targetVelocity = 0;

    private final ColorSensors colorSensors;
    private final Kickers kickers;
    private final DcMotorEx shooterLeft;
    private final DcMotorEx shooterRight;

    private double lastVelocity = -1;

    private ArrayList<ShooterCommands.ShooterCommand> queue;

    private double firingSpeed = Config.SPINNER_SPEED_NEAR;

    private boolean spinup = false, reverse = false;

    public Shooter_New(HardwareMap hardwareMap, ColorSensors colorSensors){
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        this.colorSensors = colorSensors;
        this.kickers = new Kickers(hardwareMap);

        this.shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.queue = new ArrayList<>();
    }

    /*
        Push a new command to the shooter
     */
    public void pushCommand(ShooterCommands.ShooterCommand command){
        if(command.override){
            queue.clear();
        }
        this.queue.add(command);
    }

    /*
        Update
     */
    public void update(){
        if(!queue.isEmpty()){
            ShooterCommands.ShooterCommand command = queue.get(0);

            if(!command.run(this)){
                queue.remove(command);
            }
        }

        if(spinup){
            targetVelocity = reverse ? -firingSpeed : firingSpeed;
        }else{
            targetVelocity = 0;
        }

        // Don't write to hardware ALL the time
        if(targetVelocity != lastVelocity) {
            shooterLeft.setVelocity(targetVelocity);
            shooterRight.setVelocity(-targetVelocity);
            lastVelocity = targetVelocity;
        }
    }

    /*
        Setters
     */
    public void setTargetVelocity(double velocity){
        targetVelocity = velocity;
    }

    public void setPID(PIDFCoefficients pid){
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
    }

    public void setFiringSpeed(double speed){
        this.firingSpeed = speed;
    }

    public void setSpinup(boolean value, boolean reverse){
        this.spinup = value;
        this.reverse = reverse;
    }

    /*
        Getters
     */
    public double getVelocity(){
        return (Math.abs(shooterLeft.getVelocity()) + Math.abs(shooterRight.getVelocity())) / 2.0;
    }

    public double getLeftVelocity(){
        return Math.abs(shooterLeft.getVelocity());
    }

    public double getRightVelocity(){
        return Math.abs(shooterRight.getVelocity());
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

    public Kickers getKickers(){
        return kickers;
    }

    public ColorSensors getColorSensors(){
        return colorSensors;
    }

    public String getCommandStackString(){
        String stack = "";
        for(int i = 0; i < 10; i++){
            if(queue.size() > i)
                stack += "[ " + queue.get(i).toString() + " ]\n";
            else stack += "[         ]";
        }
        return stack;
    }

    public double getFiringSpeed(){
        return firingSpeed;
    }
}
