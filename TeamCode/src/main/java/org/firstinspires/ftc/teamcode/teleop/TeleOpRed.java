package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** TeleOp Main for the red alliance. */
@TeleOp(name = "TeleOp Red", group = "Main")
public class TeleOpRed extends TeleOpMain {

    @Override
    protected boolean onBlueAlliance() {
        return false;
    }
}
