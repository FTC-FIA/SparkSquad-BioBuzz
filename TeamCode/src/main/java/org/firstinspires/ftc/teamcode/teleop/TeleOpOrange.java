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

@TeleOp(name = "TeleOp Orange", group = "Main")
public class TeleOpOrange extends OrangeBotBaseOpMode {

    private boolean intakeOn = false;
    private boolean reverse = false;
    private double intakePower = 0.1;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        waitForStart();

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.xWasPressed()) {
                reverse = !reverse;
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


            double thePower = reverse ? -intakePower : intakePower;
            intake.setPower(intakeOn ? thePower : 0.0);

            double position = intake.getCurrentPosition();

            if (gamepad1.aWasPressed()) {
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake.setTargetPosition(1450);
                intake.setPower(intakePower);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("Current position", position);
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }

    }


}