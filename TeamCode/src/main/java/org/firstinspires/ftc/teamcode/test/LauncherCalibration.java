package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import java.util.ArrayDeque;

/**
 * One-gamepad tool for building the distance -> launcher velocity table.
 *
 * Use it with the Launcher Calibration Sheet, which lists the spots to shoot from and has room
 * to write down the results. Drive to each spot by watching the Pose line, aim, and try
 * velocities until you find the lowest and highest that still score.
 *
 * Place the robot at the Auton1 start before INIT: odometry starts there.
 * Load balls by hand (no intake). Line the turret up facing straight forward before INIT.
 */
@TeleOp(name = "Launcher Calibration", group = "Test")
public class LauncherCalibration extends GreenBotBaseOpMode {

    // Odometry starts here (the Auton1 start pose): place the robot here before INIT.
    private static final double START_X = 63;
    private static final double START_Y = -18;
    private static final double START_HEADING = 180;

    private static final double START_VELOCITY = Launcher.SHOOT_VELOCITY;
    private static final double BIG_STEP = 50;     // D-pad up/down
    private static final double SMALL_STEP = 10;   // D-pad right/left

    private static final double DRIVE_SPEED = 0.3; // slow, for driving to each spot
    private static final double TURRET_NUDGE_POWER = 0.25;
    private static final double FEEDER_POWER = 1.0;

    private static final int DISTANCE_SAMPLES = 20; // readings averaged for the distance shown

    private double velocity = START_VELOCITY;
    private boolean launcherOn = true;

    private final ArrayDeque<Double> recentDistances = new ArrayDeque<>();

    @Override
    public void runOpMode() {
        config();
        aimer.resetTurretEncoder(); // turret should face straight forward now
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        while (opModeInInit()) {
            if (gamepad1.x) aimer.setBlueAlliance(true);
            if (gamepad1.b) aimer.setBlueAlliance(false);
            telemetry.addData("Alliance", "%s   (X = BLUE, B = RED)",
                    aimer.isBlueAlliance() ? "BLUE" : "RED");
            telemetry.addLine("Turret encoder zeroed -- it should be facing straight forward.");
            drive.showPose(); // should read x 63, y -18, heading 180
            telemetry.update();
        }

        waitForStart();
        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();

        while (opModeIsActive()) {
            // ---------- Drive (slow) ----------
            drive.drive(
                    -gamepad1.left_stick_y * DRIVE_SPEED,
                    gamepad1.left_stick_x * DRIVE_SPEED,
                    -gamepad1.right_stick_x * DRIVE_SPEED);

            // ---------- Turret: hold Y to auto-aim, bumpers to nudge ----------
            aimer.update();
            if (gamepad1.left_bumper) {
                aimer.moveTurret(-TURRET_NUDGE_POWER);
            } else if (gamepad1.right_bumper) {
                aimer.moveTurret(TURRET_NUDGE_POWER);
            } else if (gamepad1.y) {
                aimer.aim();
            } else {
                aimer.stopTurret();
            }

            // ---------- Launcher: A toggles, D-pad sets velocity ----------
            if (gamepad1.aWasPressed()) launcherOn = !launcherOn;
            if (gamepad1.dpadUpWasPressed()) velocity += BIG_STEP;
            if (gamepad1.dpadDownWasPressed()) velocity -= BIG_STEP;
            if (gamepad1.dpadRightWasPressed()) velocity += SMALL_STEP;
            if (gamepad1.dpadLeftWasPressed()) velocity -= SMALL_STEP;
            velocity = Range.clip(velocity,
                    Launcher.MIN_TARGET_VELOCITY, Launcher.MAX_TARGET_VELOCITY);

            if (launcherOn) launcher.setTargetVelocity(velocity);
            else launcher.stop();

            // ---------- Feeder: hold right trigger to shoot, hold B to back out ----------
            if (gamepad1.right_trigger > 0.5) feeder.setPower(FEEDER_POWER);
            else if (gamepad1.b) feeder.setPower(-FEEDER_POWER);
            else feeder.setPower(0);

            updateLights();

            // ---------- Distance: average recent readings so the number holds still ----------
            if (aimer.hasTarget()) {
                recentDistances.addLast(aimer.getDistanceInches());
                if (recentDistances.size() > DISTANCE_SAMPLES) recentDistances.removeFirst();
            } else {
                recentDistances.clear(); // don't show old readings
            }
            double distance = averageDistance();

            showTelemetry(distance);
        }

        drive.stopDriving();
        aimer.stop();
        launcher.stop();
        feeder.setPower(0);
    }

    private double averageDistance() {
        if (recentDistances.isEmpty()) return 0;
        double sum = 0;
        for (double d : recentDistances) sum += d;
        return sum / recentDistances.size();
    }

    private void showTelemetry(double distance) {
        drive.showPose();
        if (recentDistances.isEmpty()) {
            telemetry.addData("Distance", "NO HIVE IN VIEW");
        } else {
            telemetry.addData("Distance (in)", "%.1f   (raw %.1f, %d tags)",
                    distance, aimer.getDistanceInches(), aimer.getTagCount());
        }
        telemetry.addData("Velocity", "%.0f %s", velocity, launcherOn ? "" : "(launcher OFF)");
        launcher.showStatus();
        telemetry.addData("Aim", "%s   (tx %.2f deg)",
                aimer.isAimed() ? "LINED UP" : (aimer.hasTarget() ? "not lined up" : "no hive"),
                aimer.getTxDegrees());
        telemetry.addData("Turret", aimer.getStatus());
        telemetry.addLine();
        telemetry.addLine("Sticks: drive (slow)   Y (hold): auto-aim   Bumpers: turret");
        telemetry.addLine(String.format("D-pad Up/Down: +/-%.0f   Right/Left: +/-%.0f   A: launcher on/off",
                BIG_STEP, SMALL_STEP));
        telemetry.addLine("Right trigger (hold): feed   B (hold): feeder reverse");
        telemetry.update();
    }
}
