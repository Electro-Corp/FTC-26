package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Set;

public class ShooterCommands {
    public static abstract class ShooterCommand{
        public boolean loop = false, block = false, override = false;
        public abstract boolean run(Shooter_New shooter);
        public abstract String toString();
    }

    // Commands
    public static class SpinUp extends ShooterCommand{
        private boolean fast, reverse;
        public SpinUp(boolean fast, boolean reverse){
            this.fast = fast;
            this.reverse = reverse;
        }

        @Override
        public boolean run(Shooter_New shooter){
            shooter.setSpinup(true, reverse);
            return false;
        }

        @Override
        public String toString() { return "SpinUp"; }
    }
    public static class StopCommand extends ShooterCommand{
        public StopCommand(){
            this.override = true;
        }

        @Override
        public boolean run(Shooter_New shooter){
            shooter.setSpinup(false, false);
            shooter.getKickers().retractKicker(Kickers.Position.LEFT);
            shooter.getKickers().retractKicker(Kickers.Position.MID);
            shooter.getKickers().retractKicker(Kickers.Position.RIGHT);
            return false;
        }

        @Override
        public String toString() { return "Stop"; }
    }
    public static class ShootCommand extends ShooterCommand{
        Kickers.Position pos;

        public ShootCommand(Kickers.Position pos){
            this.pos = pos;
            this.loop = true;
        }

        public ShootCommand(Kickers.Position pos, boolean waitForSpeed){
            this.pos = pos;
            this.loop = waitForSpeed;
        }

        @Override
        public boolean run(Shooter_New shooter){
            if(loop){
                if((Math.abs(shooter.getTargetVelocity()) - 10 < shooter.getVelocity() && Math.abs(shooter.getTargetVelocity()) + 10 > shooter.getVelocity())){
                    shooter.getKickers().fireKicker(pos);
                    shooter.pushCommand(new RetractKickerCommand(pos));
                    return false;
                }
                return true;
            }
            shooter.getKickers().fireKicker(pos);
            shooter.pushCommand(new RetractKickerCommand(pos));
            return false;
        }

        @Override
        public String toString() { return "Shoot"; }
    }

    public static class ShootThreeCommand extends ShooterCommand{
        @Override
        public boolean run(Shooter_New shooter) {
            shooter.pushCommand(new ShootCommand(Kickers.Position.LEFT));
            shooter.pushCommand(new DelayCommand(250));
            shooter.pushCommand(new ShootCommand(Kickers.Position.MID, false));
            shooter.pushCommand(new DelayCommand(250));
            shooter.pushCommand(new ShootCommand(Kickers.Position.RIGHT, false));
            return false;
        }

        @Override
        public String toString() { return "ShootThree"; }
    }

    public static class ShootColorCommand extends ShooterCommand{
        BallColor target;
        public ShootColorCommand(BallColor color){
            this.target = color;
        }

        @Override
        public boolean run(Shooter_New shooter){
            ShootCommand shootCommand = new ShootCommand(shooter.getColorSensors().getPositionOfColor(target));
            return shootCommand.run(shooter);
        }

        @Override
        public String toString() { return "ShootColor"; }
    }

    public static class RetractKickerCommand extends ShooterCommand{
        private Kickers.Position pos;
        public RetractKickerCommand(Kickers.Position pos){
            this.loop = true;
            this.pos = pos;
        }

        @Override
        public boolean run(Shooter_New shooter) {
            if((Math.abs(shooter.getTargetVelocity()) - 10 < shooter.getVelocity() && Math.abs(shooter.getTargetVelocity()) + 10 > shooter.getVelocity())){
                return true;
            }
            shooter.getKickers().retractKicker(pos);
            return false;
        }

        @Override
        public String toString() { return "RetractKicker"; }
    }

    // No reason to exist besides I like the Command system
    public static class SetPIDCommand extends ShooterCommand{
        PIDFCoefficients pid;
        public SetPIDCommand(PIDFCoefficients pid){
            this.pid = pid;
        }

        @Override
        public boolean run(Shooter_New shooter) {
            shooter.setPID(pid);
            return false;
        }

        @Override
        public String toString() {
            return "SetPIDCommand";
        }
    }

    public static class DelayCommand extends ShooterCommand{
        private double delay, startTime = -1;
        public DelayCommand(double delay){
            this.delay = delay;
        }

        @Override
        public boolean run(Shooter_New shooter) {
            if(startTime == -1) {
                startTime = System.currentTimeMillis();
            }else if(System.currentTimeMillis() - startTime > delay){
                return false;
            }
            return true;
        }

        @Override
        public String toString() {
            return "Delay";
        }
    }
}
