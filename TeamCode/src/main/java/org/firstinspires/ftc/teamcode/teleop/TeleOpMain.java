package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

import java.util.ArrayDeque;

@TeleOp(name = "TeleOp Main", group = "Main")
public class TeleOpMain extends GreenBotBaseOpMode {

    private final double INITIAL_LAUNCHER_TARGET_VELOCITY = 1250;
    private final double LAUNCHER_VELOCITY_STEP = 50;

    private final double INTAKE_POWER = 1.0;
    private final double FEEDER_POWER = 1.0;

    private final double SPEED_FACTOR = 0.5;

    private final double TURRET_NUDGE_POWER = 0.25;
    private final double TURRET_ALIGN_POWER = 0.15;     // slower d-pad nudge while holding Guide
    private final double TURRET_ZERO_HOLD_TIME = 0.5;   // s Guide must be held before release zeroes
    private final int DISTANCE_SAMPLES = 10;            // hive distance readings averaged for launcher speed

    private boolean intakeOn = true;
    private boolean wasOnTarget = false;
    private boolean intakeReversed = false;
    private boolean aligningTurret = false;            // driver 2 is holding Guide
    private final ElapsedTime alignTimer = new ElapsedTime();
    private final ArrayDeque<Double> recentDistances = new ArrayDeque<>(); // while auto-aiming

    @Override
    public void runOpMode() { // when driver hits INIT
        config();

        // Auton ends with the turret facing straight forward, so that position becomes 0.
        // If it isn't lined up, driver 2 can re-zero it during the match (hold Guide, see below).
        aimer.resetTurretEncoder();

        // Pick the alliance during INIT; the LEDs show the alliance color until START.
        while (opModeInInit()) {
            selectAlliance();
            helpMenu();
            telemetry.update();
        }

        waitForStart();
        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();
        launcher.setTargetVelocity(INITIAL_LAUNCHER_TARGET_VELOCITY);

        while (opModeIsActive()) {
            /////////////////////////
            // handle driving - driver 1
            /////////////////////////
            double forward = -gamepad1.left_stick_y * SPEED_FACTOR;
            double strafe = gamepad1.left_stick_x * SPEED_FACTOR;
            double turn = -gamepad1.right_stick_x * SPEED_FACTOR;
            drive.drive(forward, strafe, turn);

            /////////////////////////
            // handle intake - driver 1
            /////////////////////////
            if (gamepad1.leftBumperWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.rightBumperWasPressed()) {
                intakeReversed = !intakeReversed;
            }
            double intakePower;
            intakePower = intakeOn ? INTAKE_POWER : 0.0;
            intakePower = intakeReversed ? -intakePower : intakePower;
            intake.setPower(intakePower);

            /////////////////////////
            // handle wedge - driver 1
            /////////////////////////
            if (gamepad1.dpadUpWasPressed()) {
                wedgeServo.setPosition(WEDGE_UP);
            }
            if (gamepad1.dpadDownWasPressed()) {
                wedgeServo.setPosition(WEDGE_DOWN);
            }

            /////////////////////////
            // handle feeder - driver 2
            /////////////////////////
            if (gamepad2.leftBumperWasPressed()) {
                feeder.setPower(FEEDER_POWER);
            } else if (gamepad2.rightBumperWasPressed()) {
                feeder.setPower(-FEEDER_POWER);
            } else if (gamepad2.aWasPressed()){
                feeder.setPower(0.0);
            }

            /////////////////////////
            // handle turret - driver 2
            // D-pad L/R nudges (within the soft limits) and overrides auto-aim.
            // Hold Y to auto-aim at the hive. While it sees the hive, auto-aim also sets the
            // launcher speed from the calibration table (Launcher.CAL_DISTANCES/CAL_VELOCITIES),
            // using the average of the last DISTANCE_SAMPLES distance readings because single
            // readings are noisy. Let go of Y and the launcher stays at that speed; d-pad up/down
            // still adjusts it from there.
            //
            // Re-zeroing: hold Guide (the center button) and use d-pad L/R to line the turret
            // up facing straight forward -- this nudge IGNORES the soft limits, since they're
            // wrong if the zero is. Release Guide and that position becomes forward (0). Guide
            // has to be held at least TURRET_ZERO_HOLD_TIME, so a quick bump does nothing.
            /////////////////////////
            aimer.update();
            boolean autoAiming = false;
            if (gamepad2.guide) {
                if (!aligningTurret) {
                    aligningTurret = true;
                    alignTimer.reset();
                }
                if (gamepad2.dpad_left || gamepad2.dpad_right) {
                    aimer.nudgeTurret(gamepad2.dpad_right ? TURRET_ALIGN_POWER : -TURRET_ALIGN_POWER);
                } else {
                    aimer.stopTurret();
                }
            } else if (aligningTurret) {
                // Guide just released
                aligningTurret = false;
                aimer.stopTurret();
                if (alignTimer.seconds() >= TURRET_ZERO_HOLD_TIME) {
                    aimer.resetTurretEncoder();
                    gamepad2.rumble(500); // tell driver 2 it took
                }
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                aimer.moveTurret(gamepad2.dpad_right ? TURRET_NUDGE_POWER : -TURRET_NUDGE_POWER);
            } else if (gamepad2.y) {
                aimer.aim();
                autoAiming = true;
                if (aimer.hasTarget()) {
                    recentDistances.addLast(aimer.getDistanceInches());
                    if (recentDistances.size() > DISTANCE_SAMPLES) recentDistances.removeFirst();
                    launcher.setTargetVelocityForDistance(averageDistance());
                }
            } else {
                aimer.stopTurret();
            }
            // Start the distance average fresh on the next aim: the robot will have moved.
            if (!autoAiming) recentDistances.clear();

            // Buzz driver 2's controller once when the turret first lines up.
            boolean onTarget = aimer.isOnTarget();
            if (onTarget && !wasOnTarget) {
                gamepad2.rumble(200);
            }
            wasOnTarget = onTarget;

            /////////////////////////
            // handle launcher - driver 2
            // (runs continuously)
            /////////////////////////
            if (gamepad2.dpadUpWasPressed()) {
                launcher.changeTargetVelocity(LAUNCHER_VELOCITY_STEP);
            }
            if (gamepad2.dpadDownWasPressed()) {
                launcher.changeTargetVelocity(-LAUNCHER_VELOCITY_STEP);
            }

            /////////////////////////
            // update LEDs
            /////////////////////////
            updateLights();

            /////////////////////////
            // update telemetry
            /////////////////////////
            if (gamepad1.start || gamepad2.start) {
                helpMenu();
            } else if (aligningTurret) {
                telemetry.addLine("TURRET ZERO: d-pad L/R until the turret faces straight forward,");
                telemetry.addLine("then release Guide. (Soft limits are OFF - watch the wires.)");
                telemetry.addData("Turret ticks", aimer.getTurretTicks());
            } else {
                launcher.showStatus();
                aimer.showStatus();
                drive.showPose();
            }
            telemetry.update();
        }

        drive.stopDriving();
        aimer.stop();
        launcher.stop();
    }

    private void helpMenu() {
        telemetry.addData("Help Menu (this)", "Either Gamepad - Hold Start");
        telemetry.addData("", "");

        telemetry.addData("DRIVER 1", "Controls");
        telemetry.addData("Sticks", "Standard Mecanum");
        telemetry.addData("L Bumper", "Intake ON/OFF (Toggle)");
        telemetry.addData("R Bumper", "Intake Reverse (Toggle)");
        telemetry.addData("DPad Up/Down", "Wedge Up/Down");
        telemetry.addData("", "");

        telemetry.addData("DRIVER 2", "Controls");
        telemetry.addData("Y (Hold)", "Turret Auto-Aim + launcher speed from distance");
        telemetry.addData("DPad L/R", "Turret L/R");
        telemetry.addData("Guide (Hold)", "Turret zero: DPad L/R to face fwd, release");
        telemetry.addData("L Bumper", "Feeder SHOOT (Keep On)");
        telemetry.addData("R Bumper", "Feeder REVERSE (Keep On)");
        telemetry.addData("A", "Feeder OFF");
        telemetry.addData("DPad Up/Down", "Launcher +/-50");
    }

    /** Average of the recent hive distances (inches). Only call when there is at least one. */
    private double averageDistance() {
        double sum = 0;
        for (double d : recentDistances) sum += d;
        return sum / recentDistances.size();
    }
}