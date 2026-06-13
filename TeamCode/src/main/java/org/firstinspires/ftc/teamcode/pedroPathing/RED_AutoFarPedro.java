package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Far Pedro - RED", group = "Autonomous")
public class RED_AutoFarPedro extends AutoFarPedro {
    @Override
    protected boolean isBlue() {
        return false;
    }
}
