package org.firstinspires.ftc.teamcode.opsmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//testing 123

@TeleOp(name= "A_BLUE TeleOp")
public class BLUE_TeleOp extends MainTeleOp {

    @Override
    protected int GetSideMultiplier() {
        return -1;
    }
}
