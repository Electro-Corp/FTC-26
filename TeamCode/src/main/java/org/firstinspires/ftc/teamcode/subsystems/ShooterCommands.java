package org.firstinspires.ftc.teamcode.subsystems;

public class ShooterCommands {
    public static abstract class ShooterCommand{
        public boolean loop = false, block = false;
        public abstract boolean run(Shooter_New shooter);
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
            shooter.setTargetVelocity(reverse ? -Shooter_New.Config.SPINNER_SPEED_NEAR : Shooter_New.Config.SPINNER_SPEED_NEAR);
            return false;
        }
    }
    public static class StopCommand extends ShooterCommand{
        @Override
        public boolean run(Shooter_New shooter){
            shooter.setTargetVelocity(0);
            return false;
        }
    }
    public static class ShootCommand extends ShooterCommand{
        Kickers.Position pos;

        public ShootCommand(Kickers.Position pos){
            this.pos = pos;
            this.loop = false;
        }

        public ShootCommand(Kickers.Position pos, boolean waitForSpeed){
            this.pos = pos;
            this.loop = waitForSpeed;
        }

        @Override
        public boolean run(Shooter_New shooter){
            if(loop){
                if(shooter.getVelocity() - 10 < shooter.getTargetVelocity() && shooter.getTargetVelocity() < shooter.getVelocity() + 10){
                    shooter.getKickers().fireKicker(pos);
                    return false;
                }
                return true;
            }
            shooter.getKickers().fireKicker(pos);
            shooter.pushCommand(new RetractKickerCommand(pos));
            return false;
        }
    }

    public static class ShootColorCommand extends ShooterCommand {
        BallColor target;
        public ShootColorCommand(BallColor color){
            this.target = color;
        }

        @Override
        public boolean run(Shooter_New shooter){
            ShootCommand shootCommand = new ShootCommand(shooter.getColorSensors().getPositionOfColor(target)); // replace with real value
            return shootCommand.run(shooter);
        }
    }

    public static class RetractKickerCommand extends ShooterCommand{
        private Kickers.Position pos;
        public RetractKickerCommand(Kickers.Position pos){
            this.loop = true;
            this.pos = pos;
        }

        @Override
        public boolean run(Shooter_New shooter) {
            if(shooter.getVelocity() - 10 < shooter.getTargetVelocity() && shooter.getTargetVelocity() < shooter.getVelocity() + 10){
                return true;
            }
            shooter.getKickers().fireKicker(pos);
            return false;
        }
    }
}
