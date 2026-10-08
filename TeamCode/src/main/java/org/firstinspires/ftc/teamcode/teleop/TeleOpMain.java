package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpMain extends OpMode {

    // Declare OpMode members.
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    private final int INITIAL_LAUNCHER_TARGET_VELOCITY = 1250;
    private final int MIN_LAUNCHER_TARGET_VELOCITY = 200;
    private final int MAX_LAUNCHER_TARGET_VELOCITY = 2000;

    private final double INTAKE_POWER = 1.0;
    private final double WINDMILL_POWER = 1.0;

    private boolean intakeOn = false;
    private double launcherTargetVelocity = INITIAL_LAUNCHER_TARGET_VELOCITY;

    @Override
    public void init() { // when driver hits INIT

        // instantiate hardware components
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        // config hardware components
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5) // adjust these for spinup and recovery
        );

        // init servos - start off
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        helpMenu();
        telemetry.update();
    }

    @Override
    public void start() {
        intakeOn = true;
    }

    @Override
    public void loop() {

        /////////////////////////
        // handle driving
        /////////////////////////
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        mecanumDrive(forward, strafe, turn);

        /////////////////////////
        // handle intake
        /////////////////////////
        if (gamepad1.leftBumperWasPressed()) {
            intakeOn = !intakeOn;
        }
        double intakePower = 0;
        if (intakeOn) {
            intakePower = INTAKE_POWER;
        } // else stays at 0
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        /////////////////////////
        // handle launcher - runs continuously
        /////////////////////////
        if (gamepad1.dpadUpWasPressed()) {
            launcherTargetVelocity += 100;
            if (launcherTargetVelocity > MAX_LAUNCHER_TARGET_VELOCITY) {
                launcherTargetVelocity = MAX_LAUNCHER_TARGET_VELOCITY;
            }
        }
        if (gamepad1.dpadDownWasPressed()) {
            launcherTargetVelocity -= 100;
            if (launcherTargetVelocity < MIN_LAUNCHER_TARGET_VELOCITY) {
                launcherTargetVelocity = MIN_LAUNCHER_TARGET_VELOCITY;
            }
        }
        launcher.setVelocity(launcherTargetVelocity);

        /////////////////////////
        // handle windmill
        /////////////////////////
        if (gamepad1.right_bumper) {
            windmillServo.setPower(WINDMILL_POWER);
        } else {
            windmillServo.setPower(0);
        }

        /////////////////////////
        // update telemetry
        /////////////////////////
        if (gamepad1.start) {
            helpMenu();
        } else {
            double launcherVelocity = launcher.getVelocity();
            telemetry.addData("Launcher - Target Velocity", launcherTargetVelocity);
            telemetry.addData("Launcher - Current Velocity", launcherVelocity);
            telemetry.addData("Windmill Power", windmillServo.getPower());
            telemetry.addData("Left Intake Servo", leftIntakeServo.getPower());
            telemetry.addData("Right Intake Servo", rightIntakeServo.getPower());
        }
        telemetry.update();

        //TODO: Update Panels

    }

    @Override
    public void stop() {
    }

    private void mecanumDrive(double forward, double strafe, double turn) {
        double fl = forward + strafe - turn;
        double bl = forward - strafe - turn;
        double fr = forward - strafe + turn;
        double br = forward + strafe + turn;

        // Scale all four down together if any exceeds 1, so the direction is kept, just slower.
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private void helpMenu() {
        telemetry.addData("Drive controls", "Standard Mecanum");
        telemetry.addData("Intake Toggle", "Left Bumper");
        telemetry.addData("Run Windmill", "Right Bumper");
        telemetry.addData("Launcher +", "DPad Up");
        telemetry.addData("Launcher - ", "DPad Down");
    }
}