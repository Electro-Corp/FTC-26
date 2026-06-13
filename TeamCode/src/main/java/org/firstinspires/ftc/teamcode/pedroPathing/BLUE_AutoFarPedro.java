package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Far Pedro - BLUE", group = "Autonomous")
public class BLUE_AutoFarPedro extends AutoFarPedro {
    @Override
    protected boolean isBlue() {
        return true;
    }
}
