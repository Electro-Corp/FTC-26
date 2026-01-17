package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="A_RED Auto")
public class RED_Auto extends AutoRoot{

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
}
