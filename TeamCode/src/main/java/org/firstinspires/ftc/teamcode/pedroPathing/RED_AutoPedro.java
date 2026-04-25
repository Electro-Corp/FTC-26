package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Pedro - RED", group = "Autonomous")
public class RED_AutoPedro extends AutoPedro {
    @Override
    protected boolean isBlue() {
        return false;
    }
}
