package org.firstinspires.ftc.teamcode.subsystems;

public enum BallColor {
    PURPLE,
    GREEN,
    UNKNOWN;

    public String toString() {
        switch (this) {
            case PURPLE:
                return "PURPLE";
            case GREEN:
                return "GREEN";
            default:
                return "NONE";
        }
    }
}
