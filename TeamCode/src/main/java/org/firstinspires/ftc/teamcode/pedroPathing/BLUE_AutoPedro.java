package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Pedro - BLUE", group = "Autonomous")
public class BLUE_AutoPedro extends AutoPedro {
    @Override
    protected boolean isBlue() {
        return true;
    }
}
