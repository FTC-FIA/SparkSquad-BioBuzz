package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.OrangeBotBaseOpMode;

@Autonomous(name = "Auton - Dami", group = "Auton")
public class AutonDami extends OrangeBotBaseOpMode {

    // Where to go, in field coordinates. The only numbers you change to go somewhere else.
    static final double START_X = 63;          // left side against the red wall
    static final double START_Y = -12;
    static final double START_HEADING = 180;    // facing -x, along the wall toward the zone

    static final double LEAVE_X = 62;          // move 1: straight out from the wall, no turning
    static final double LEAVE_Y = -12;
    static final double LEAVE_HEADING =  180;

    static final double TURN_X =  62;           // move 2: drive and turn at the same time
    static final double TURN_Y = -12;
    static final double TURN_HEADING = 180;      // end up facing +y, out into the field

    static final double PARK_X = -36;           // move 3: down into the loading zone
    static final double PARK_Y = -60;
    static final double PARK_HEADING =  0;

    // How we drive. One speed the whole way, same as the timed version.
    static final double DRIVE_POWER = 1;      // how hard to push, 0 to 1
    static final double TURN_POWER = 1;       // how hard to spin, 0 to 1
    static final double TOLERANCE = 0;        // inches; "close enough, stop"
    static final double HEADING_TOLERANCE = 3;  // degrees; "pointed close enough, stop turning"
    static final double TIMEOUT = 5.0;          // seconds; give up rather than hang forever

    @Override
    public void runOpMode() {

        config();


        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y,
                AngleUnit.DEGREES, START_HEADING));

        telemetry.addData("Ready", "Press START");
        drive.showPose();

        waitForStart();
        drive.driveTo(LEAVE_X, LEAVE_Y, LEAVE_HEADING);   // out from the wall -- scores LEAVE
        drive.driveTo(TURN_X, TURN_Y, TURN_HEADING);      // drive and turn together -- the thing to watch

        // TODO: use named variables for all coordinates
        drive.driveTo(50, -12, 180);
        sleep(3000);
        drive.driveTo(46, -12, -90);
        drive.driveTo(24, -55, -90);
        sleep(3000);
        drive.driveTo(-48, -48, -90);
        drive.driveTo(-45, -15, 0);
        sleep(3000);
        drive.driveTo(-55, -24, 180);
        sleep(3000);
        drive.driveTo(-48, -15, 0);
        sleep(3000);
        drive.driveTo(PARK_X, PARK_Y, PARK_HEADING);      // into the loading zone -- scores PARK

        drive.showPose();
        sleep(3000);
    }

}
