package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "Auton 2: Pinpoint", group = "Main")
public class AutonPinpointNoTurns extends LinearOpMode {

    // Where to go, in field coordinates. The only numbers you change to go somewhere else.
    static final double START_X = -64.5;          // left side against the red wall
    static final double START_Y = 12;
    static final double START_HEADING = 0;    // facing -x, along the wall toward the zone

    static final double LEAVE_X = -59.5;          // move 1: off the wall
    static final double LEAVE_Y = 12;

    static final double PARK_X = -59.5;           // move 2: into the loading zone
    static final double PARK_Y = 18;

    // How we drive. One speed the whole way, same as the timed version.
    static final double DRIVE_POWER = 0.2;      // how hard to push, 0 to 1
    static final double TOLERANCE = 0.5;        // inches; "close enough, stop"
    static final double TIMEOUT = 5.0;          // seconds; give up rather than hang forever

    DcMotor frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-7.375, -5.875, DistanceUnit.INCH);

        // On a real robot, also call setOffsets(), setEncoderResolution() and setEncoderDirections()
        // here to describe how the odometry pods are mounted. The simulator does not need them.

        // The important line. The Pinpoint only knows how far it has rolled since you last told it
        // something, so tell it where it is starting. Now every pose it reports is a field coordinate.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Ready", "Press START");
        showPose();

        waitForStart();

        driveTo(LEAVE_X, LEAVE_Y);   // scores LEAVE
        driveTo(PARK_X, PARK_Y);     // scores PARK

        showPose();
        sleep(3000);
    }

    /**
     * Head for a spot on the field at DRIVE_POWER, and cut the motors once we are close enough.
     * Full speed, then off, nothing in between -- watching where that leaves the robot is the point.
     */
    private void driveTo(double targetX, double targetY) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < TIMEOUT) {

            pinpoint.update();                      // nothing updates until you call this
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - x;            // how far off we are, in FIELD coordinates
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            showPose();
            telemetry.addData("Target", "x: %.1f  y: %.1f   (%.1f in away)", targetX, targetY, distance);
            telemetry.update();

            // The whole improvement over the timed version: what we check is our position, not a clock.
            if (distance < TOLERANCE) break;

            driveFieldRelative(errorX, errorY, heading, DRIVE_POWER, 0);
        }

        stopDriving();
        sleep(500);   // let the robot coast to a stop before the next move reads the odometry
    }

    /**
     * Drive toward a direction measured on the FIELD, whichever way the robot happens to be pointed.
     *
     * The motors only understand "forward" and "right," and those swing around as the robot turns.
     * Rotating the field vector by the robot's heading converts one into the other -- it is a plain
     * 2-D rotation. Dividing by the length throws away the distance and keeps only the direction;
     * power decides the speed.
     *
     * @param fieldX  x part of the direction to travel, in field coordinates
     * @param fieldY  y part of the direction to travel, in field coordinates
     * @param heading robot's current heading, degrees
     * @param power   how hard to push, 0 to 1
     * @param turn    spin power, + is counterclockwise
     */
    private void driveFieldRelative(double fieldX, double fieldY, double heading, double power, double turn) {
        double h = Math.toRadians(heading);
        double length = Math.hypot(fieldX, fieldY);

        double forward = (fieldX * Math.cos(h) + fieldY * Math.sin(h)) / length;
        double right = (fieldX * Math.sin(h) - fieldY * Math.cos(h)) / length;

        drive(forward * power, right * power, turn);
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

    /** Everybody stop. */
    private void stopDriving() {
        drive(0, 0, 0);
    }

    /** Put the robot's current field position on the driver station. */
    private void showPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pose", "x: %.1f  y: %.1f  heading: %.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
    }
}
