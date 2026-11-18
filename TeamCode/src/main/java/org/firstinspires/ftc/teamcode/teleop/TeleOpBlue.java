package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** TeleOp Main for the blue alliance. */
@TeleOp(name = "TeleOp Blue", group = "Main")
public class TeleOpBlue extends TeleOpMain {

    @Override
    protected boolean onBlueAlliance() {
        return true;
    }
}
