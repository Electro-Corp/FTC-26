package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BLUE Auto")
public class BLUE_Auto extends AutoRoot{
    @Override
    protected int getTargetTag() {
        return 20;
    }
}
