package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.SimBotBaseOpMode;
import org.firstinspires.ftc.teamcode.sim.SimStart;

@Autonomous(name = "Auton 1st Shooter - Abram", group = "Auton")
public class AutonOneAbram extends SimBotBaseOpMode {

    // Where to go, in field coordinates. The only numbers you change to go somewhere else.
    static final double START_X = 63.5;          // left side against the red wall
    static final double START_Y = -12;
    static final double START_HEADING = 180;    // facing -x, along the wall toward the zone

    static final double SHOOT_X = 60;           // move 2: into the loading zone
    static final double SHOOT_Y = -12;

    static final double PARK_X = 24;           // move 2: into the loading zone
    static final double PARK_Y = -57;

    static final double PARK2_X = -24;           // move 2: into the loading zone
    static final double PARK2_Y = -57;

    static final double PARK3_X = -20;           // move 2: into the loading zone
    static final double PARK3_Y = -63;

    // How we drive. One speed the whole way, same as the timed version.
    static final double DRIVE_POWER = 0.8;      // how hard to push, 0 to 1
    static final double TOLERANCE = 0.5;        // inches; "close enough, stop"
    static final double TIMEOUT = 5.0;          // seconds; give up rather than hang forever

    DcMotor frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        config();

        // SIMULATOR ONLY
        SimStart.setPose(START_X, START_Y, START_HEADING);
        sleep(100);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Ready", "Press START");
        drive.showPose();

        waitForStart();

        // Abram, you did not update your project before you started, so driveTo()
        // did not include the heading parameter. I added START_HEADING to all of
        // yours so I could integrate, but be sure to check this in the future!
        drive.driveTo(SHOOT_X, SHOOT_Y, START_HEADING);
        sleep(5000);
        drive.driveTo(PARK_X, PARK_Y, START_HEADING);     // scores PARK
        sleep(5000);
        drive.driveTo(PARK2_X, PARK2_Y, START_HEADING);     // scores PARK
        drive.driveTo(PARK3_X, PARK3_Y, START_HEADING);

        drive.showPose();
        sleep(3000);
    }
}
