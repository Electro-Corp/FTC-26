package org.firstinspires.ftc.teamcode.opsmodes.auto;

import org.firstinspires.ftc.teamcode.subsystems.BallColor;

public enum Pattern {
    PPG(21, new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN}),
    PGP(22, new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE}),
    GPP(23, new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE});

    private int num;
    private BallColor[] colors;


    Pattern(int num, BallColor[] colors) {
        this.num = num;
        this.colors = colors;
    }

    public int getNum() {
        return num;
    }

    public BallColor[] getColors() {
        return colors;
    }

    public BallColor getColorAtIndex(int i) {
        return colors[i];
    }

    public static Pattern fromNum(int num){
        switch(num){
            case 21:
                return PPG;
            case 22:
                return PGP;
            case 23:
            default:
                return GPP;
        }
    }

    @Override
    public String toString(){
        return getNum() + " " + getColorAtIndex(0).toString() + " " + getColorAtIndex(1).toString() + " " + getColorAtIndex(2).toString();
    }
}