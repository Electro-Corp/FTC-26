package org.firstinspires.ftc.teamcode.opsmodes.auto;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public enum Pattern {
    PPG(21, new Shooter.BallColor[]{Shooter.BallColor.PURPLE, Shooter.BallColor.PURPLE, Shooter.BallColor.GREEN}),
    PGP(22, new Shooter.BallColor[]{Shooter.BallColor.PURPLE, Shooter.BallColor.GREEN, Shooter.BallColor.PURPLE}),
    GPP(23, new Shooter.BallColor[]{Shooter.BallColor.GREEN, Shooter.BallColor.PURPLE, Shooter.BallColor.PURPLE});

    private int num;
    private Shooter.BallColor[] colors;


    Pattern(int num, Shooter.BallColor[] colors) {
        this.num = num;
        this.colors = colors;
    }

    public int getNum() {
        return num;
    }

    public Shooter.BallColor[] getColors() {
        return colors;
    }

    public Shooter.BallColor getColorAtIndex(int i) {
        return colors[num];
    }

    public static Pattern fromNum(int num){
        switch(num){
            case 21:
                return GPP;
            case 22:
                return PGP;
            case 23:
                return PPG;
            default:
                return GPP;
        }
    }

    @Override
    public String toString(){
        return getNum() + " " + getColorAtIndex(0).toString() + " " + getColorAtIndex(1).toString() + " " + getColorAtIndex(2).toString();
    }
}