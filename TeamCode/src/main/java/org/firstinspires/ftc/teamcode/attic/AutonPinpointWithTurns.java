package org.firstinspires.ftc.teamcode.attic;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
@Autonomous(name = "Auton Pinpoint (w/ Heading)", group = "Main")
public class AutonPinpointWithTurns extends LinearOpMode {

    // Where to go, in field coordinates. The only numbers you change to go somewhere else.
    static final double START_X = -12;          // left side against the red wall
    static final double START_Y = -63;
    static final double START_HEADING = 180;    // facing -x, along the wall toward the zone

    static final double LEAVE_X = -12;          // move 1: straight out from the wall, no turning
    static final double LEAVE_Y = -48;
    static final double LEAVE_HEADING = 180;

    static final double TURN_X = -30;           // move 2: drive and turn at the same time
    static final double TURN_Y = -48;
    static final double TURN_HEADING = 0;       // end up facing +y, out into the field

    static final double PARK_X = -30;           // move 3: down into the loading zone
    static final double PARK_Y = -59;
    static final double PARK_HEADING = 90;

    // How we drive. One speed the whole way.
    static final double DRIVE_POWER = 0.2;      // how hard to push, 0 to 1
    static final double TURN_POWER = 0.2;       // how hard to spin, 0 to 1
    static final double TOLERANCE = 0.5;        // inches; "close enough, stop"
    static final double HEADING_TOLERANCE = 3;  // degrees; "pointed close enough, stop turning"
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
        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-7.375, -5.875, DistanceUnit.INCH);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Ready", "Press START");
        showPose();

        waitForStart();

        driveTo(LEAVE_X, LEAVE_Y, LEAVE_HEADING);   // out from the wall -- scores LEAVE
        driveTo(TURN_X, TURN_Y, TURN_HEADING);      // drive and turn together -- the thing to watch
        driveTo(PARK_X, PARK_Y, PARK_HEADING);      // into the loading zone -- scores PARK

        showPose();
        sleep(3000);
    }

    /**
     * Go to a spot on the field AND face a given direction. Driving and turning happen together,
     * each at a fixed power, and each stops on its own once it is close enough.
     */
    private void driveTo(double targetX, double targetY, double targetHeading) {

        ElapsedTime timer = new ElapsedTime();
        boolean atPosition = false;
        boolean atHeading = false;

        // The OpMode yields control to driveTo() until it's done.
        // This might not work for real auton
        while (opModeIsActive() && timer.seconds() < TIMEOUT) {

            pinpoint.update();                      // nothing updates until you call this
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - x;            // how far off we are, in FIELD coordinates
            double errorY = targetY - y;
            double distance = Math.hypot(errorX, errorY);

            // Shortest way round: 170 to -170 is a 20 degree turn, not 340.
            double headingError = AngleUnit.normalizeDegrees(targetHeading - heading);

            // Once we arrive, stay arrived. Without this the robot would chase an overshoot back
            // and forth across the target until the timeout.
            if (distance < TOLERANCE) atPosition = true;
            if (Math.abs(headingError) < HEADING_TOLERANCE) atHeading = true;

            showPose();
            telemetry.addData("Target", "x: %.1f  y: %.1f  heading: %.1f",
                    targetX, targetY, targetHeading);
            telemetry.addData("Error", "%.1f in   %.1f deg", distance, headingError);
            telemetry.update();

            if (atPosition && atHeading) break;

            double power = atPosition ? 0 : DRIVE_POWER;
            double turn = atHeading ? 0 : TURN_POWER * Math.signum(headingError);

            driveFieldRelative(errorX, errorY, heading, power, turn);
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
     * The turn is NOT rotated -- spinning means the same thing in both frames. It just rides along,
     * which is what lets the robot travel a straight line on the field while it turns.
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

        double forward = 0;                         // already on the spot? then there is no
        double right = 0;                           // direction to convert -- just turn
        if (length > 0) {
            forward = (fieldX * Math.cos(h) + fieldY * Math.sin(h)) / length;
            right = (fieldX * Math.sin(h) - fieldY * Math.cos(h)) / length;
        }

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
