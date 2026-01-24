package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Z_RED Auto Far")
public class RED_Auto_Far extends AutoRoot{

    @Override
    protected int getTargetTag() {
        return 24;
    }
    protected int getInvert(){
        return 1;
    }

    @Override
    protected double getXOffset() {
        return 0;
    }

    @Override
    protected boolean isNear() {
        return true;
    }
}
