package org.firstinspires.ftc.teamcode.opsmodes.auto;

import org.firstinspires.ftc.teamcode.subsystems.BallColor;

public enum Pattern {
    PPG(21, new BallColor.BallColor[]{BallColor.BallColor.PURPLE, BallColor.BallColor.PURPLE, BallColor.BallColor.GREEN}),
    PGP(22, new BallColor.BallColor[]{BallColor.BallColor.PURPLE, BallColor.BallColor.GREEN, BallColor.BallColor.PURPLE}),
    GPP(23, new BallColor.BallColor[]{BallColor.BallColor.GREEN, BallColor.BallColor.PURPLE, BallColor.BallColor.PURPLE});

    private int num;
    private BallColor.BallColor[] colors;


    Pattern(int num, BallColor.BallColor[] colors) {
        this.num = num;
        this.colors = colors;
    }

    public int getNum() {
        return num;
    }

    public BallColor.BallColor[] getColors() {
        return colors;
    }

    public BallColor.BallColor getColorAtIndex(int i) {
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