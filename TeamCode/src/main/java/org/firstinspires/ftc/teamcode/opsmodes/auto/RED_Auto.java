package org.firstinspires.ftc.teamcode.opsmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RED Auto")
public class RED_Auto extends AutoRoot{
    @Override
    protected int getTargetTag() {
        return 24;
    }
}
