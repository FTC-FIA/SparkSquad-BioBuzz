package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Basic TeleOp with Robot Relative Mecanum Drive
 */
@TeleOp(name = "Limelight Tester", group = "BioBuzz")
public class LimelightTester extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Limelight3A limelight;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {

        // Find the four drive motors by the names in the robot configuration.
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // The motors on the left side face the other way, so flip them.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Press START + A on your gamepad to connect it as gamepad1.");
        telemetry.addData("Ready", "Press START");
        telemetry.update();

        waitForStart();
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(9);

        // Keep going until the driver presses STOP.
        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -1.0f * gamepad1.right_stick_x;

            drive(forward, strafe, turn);
            telemetry.addData(
                    "Drive",
                    String.format(
                            "fwd: %.1f  strafe: %.1f  turn: %.1f",
                            forward,
                            strafe,
                            turn
                    )
            );

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double tx = result.getTx(); // how far left/right target is from center
                double ty = result.getTy(); // how far up/down target is from center
                double ta = result.getTa(); // size of target (% of image)

                telemetry.addData(
                        "Limelight",
                        String.format("Tx: %.2f; Ty: %.2f, Ta: %.2f", tx, ty, ta)
                );
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            telemetry.update();
        }
    }

    /**
     * Mecanum mixing: turn "go this way and spin this much" into four motor powers.
     *
     * @param forward +1 is straight out the front of the robot
     * @param right   +1 is sideways to the robot's right -- what mecanum wheels are for
     * @param turn    +1 is counterclockwise, the direction heading increases
     */
    private void drive(double forward, double right, double turn) {
        double fl = forward + right - turn;
        double bl = forward - right - turn;
        double fr = forward - right + turn;
        double br = forward + right + turn;

        // Scale all four down together if any exceeds 1, so the direction is kept, just slower.
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }
}