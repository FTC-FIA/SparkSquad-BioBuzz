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

    private final double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        waitForStart();;
        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            drive.drive(forward, strafe, turn);

            if (gamepad1.right_bumper) {
                intake.setPower(-INTAKE_POWER);
            } else {
                intake.setPower(0.0);
            }
            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0.0);
            }
        }
        telemetry.update();
    }
}