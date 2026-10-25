package org.firstinspires.ftc.teamcode.attic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;

@Disabled
@Autonomous(name = "Auton Orange", group = "Main")
public class AutonOrange extends OrangeBotBaseOpMode {

    // HARDWARE (provided by OrangeBotBaseOpMode -- be sure to call config()
    //    DcMotor frontLeft, frontRight, backLeft, backRight, intake;
    //    GoBildaPinpointDriver pinpoint;
    //    Limelight3A limelight;

    // Mehcanisms this OpMode needs
    Drive drive;

    // Driving parameters
    final double DRIVE_POWER = 0.2;      // how hard to push, 0 to 1
    final double TURN_POWER = 0.2;       // how hard to spin, 0 to 1
    final double TOLERANCE = 0.5;        // inches; "close enough, stop"
    final double HEADING_TOLERANCE = 3;  // degrees; "pointed close enough, stop turning"
    final double TIMEOUT = 5.0;          // seconds; give up rather than hang forever

    // Field coordinates for each of the spots
    final double START_X = -12;          // left side against the red wall
    final double START_Y = -63;
    final double START_HEADING = 180;    // facing -x, along the wall toward the zone

    final double LEAVE_X = -12;          // move 1: straight out from the wall, no turning
    final double LEAVE_Y = -48;
    final double LEAVE_HEADING = 180;

    final double TURN_X = -30;           // move 2: drive and turn at the same time
    final double TURN_Y = -48;
    final double TURN_HEADING = 0;       // end up facing +y, out into the field

    final double PARK_X = -30;           // move 3: down into the loading zone
    final double PARK_Y = -59;
    final double PARK_HEADING = 90;

    @Override
    public void runOpMode() {

        // call OrangeBotBaseOpMode's config() to set up hardware
        config();

        drive = new Drive(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                pinpoint,
                telemetry
        );

        // set start position so robot knows where it is
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        START_X,
                        START_Y,
                        AngleUnit.DEGREES,
                        START_HEADING
                )
        );

        // Pre-start Telmetry display
        telemetry.addData("Ready", "Press START");
        drive.showPose();

        // Hold until driver presses START
        waitForStart();

        // LET'S GOOOOO - drive to each of the spots
        drive.driveTo(LEAVE_X, LEAVE_Y, LEAVE_HEADING);
        drive.driveTo(TURN_X, TURN_Y, TURN_HEADING);
        drive.driveTo(PARK_X, PARK_Y, PARK_HEADING);
    }
}
