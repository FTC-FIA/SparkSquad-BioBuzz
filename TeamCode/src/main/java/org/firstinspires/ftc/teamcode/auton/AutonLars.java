package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;


@Autonomous(name = "Auton 1st Shooter (Lars)", group = "Auton")
public class AutonLars extends OrangeBotBaseOpMode {

    // Where to go, in field coordinates. The only numbers you change to go somewhere else.
    static final double START_X = 63;          // left side against the red wall
    static final double START_Y = -12;
    static final double START_HEADING = 180;    // facing -x, along the wall toward the zone

    static final double Garden_X = 58;          // move 1: straight out from the wall, no turning
    static final double Garden_Y = -56;
    static final double Garden_HEADING = 270;

    static final double TURN_X = 0;           // move 2: drive and turn at the same time
    static final double TURN_Y = 0;
    static final double TURN_HEADING = 0;      // end up facing +y, out into the field

    static final double PARK_X = -30;           // move 3: down into the loading zone
    static final double PARK_Y = -59;
    static final double PARK_HEADING = 90;

    // How we drive. One speed the whole way, same as the timed version.
    static final double DRIVE_POWER = 1;      // how hard to push, 0 to 1
    static final double TURN_POWER = 1;       // how hard to spin, 0 to 1
    static final double TOLERANCE = 0.5;        // inches; "close enough, stop"
    static final double HEADING_TOLERANCE = 3;  // degrees; "pointed close enough, stop turning"
    static final double TIMEOUT = 5.0;          // seconds; give up rather than hang forever

    @Override
    public void runOpMode() {

        config();

        // SIMULATOR ONLY, both lines -- delete for the real robot. The sleep gives the physics a
        // moment to actually move the robot before we read any sensors.

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Ready", "Press START");
        drive.showPose();

        waitForStart();

        drive.driveTo(Garden_X,Garden_Y,0);
        sleep(1000);

        // why drive back to start?
        drive.driveTo(START_X,START_Y, START_HEADING);
        sleep(1000);

        // into the loading zone -- scores PARK

        drive.showPose();
        sleep(3000); // don't need this prolly
    }

}
