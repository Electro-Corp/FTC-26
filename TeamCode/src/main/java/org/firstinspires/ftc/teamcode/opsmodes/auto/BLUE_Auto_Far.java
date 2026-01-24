package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z_BLUE Auto Far")
public class BLUE_Auto_Far extends AutoRoot{
    @Override
    protected int getTargetTag() {
        return 20;
    }

    protected int getInvert(){
        return -1;
    }

    @Override
    protected double getXOffset() {
        return 7.5;
    }

    @Override
    protected boolean isNear() {
        return false;
    }
}
