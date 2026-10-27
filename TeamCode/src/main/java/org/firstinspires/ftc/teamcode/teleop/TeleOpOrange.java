package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpOrange extends OrangeBotBaseOpMode {

    private boolean intakeOn = false;
    private double intakePower = 0.1;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.dpadUpWasPressed()) {
                intakePower += 0.01;
                if (intakePower > 1.0) {
                    intakePower = 1.0;
                }
            }
            if (gamepad1.dpadDownWasPressed()) {
                intakePower -= 0.01;
                if (intakePower < 0.0) {
                    intakePower = 0.0;
                }
            }
            intake.setPower(intakeOn ? intakePower : 0.0);

            telemetry.addData("Intake Power", intakePower));
        }

    }


}