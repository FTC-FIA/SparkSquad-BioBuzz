package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Aimer;
import org.firstinspires.ftc.teamcode.robot.Launcher;


/**
 * Setup: robot at START, turret turned to its auton start position (90 degrees to the side,
 * Aimer.TURRET_AUTON_START_TICKS). Press INIT, pick the alliance, check that "Turret ticks" on
 * the Driver Station matches the start position.
 */
@Autonomous(name = "S1-G-S2-PL", group = "Auton")
public class S1_G_S2_PL extends GreenBotBaseOpMode implements Locations {

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;
    static final double INTAKE_POWER = 1.0;

    // Where to point the turret at SHOOT1 before auto-aiming, in ticks from straight forward.
    // 0 = straight forward; change it if the hive isn't in the Limelight's view from there.
    static final int SHOOT1_TURRET_TICKS = 0;

    static final double TURRET_TURN_TIMEOUT = 1.5; // seconds
    static final double AIM_TIMEOUT = 1.5;        // seconds
    static final double LAUNCHER_TIMEOUT = 2.0;   // seconds

    @Override
    public void runOpMode() {
        config();
        drive.setUsePid(true);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START1_X, START1_Y,
                AngleUnit.DEGREES, START1_HEADING));

        // The turret starts turned to the side, not facing forward. Tell the aimer where it is
        // so ticks, soft limits and turnTurretTo() all count from straight forward.
        aimer.setTurretTicks(Aimer.TURRET_AUTON_START_TICKS);

        // Pick the alliance during INIT; the LEDs show the alliance color until START.
        while (opModeInInit()) {
            selectAlliance();
            drive.showPose();
            if (Aimer.TURRET_AUTON_START_TICKS == 0) {
                telemetry.addLine("WARNING: Aimer.TURRET_AUTON_START_TICKS not set yet");
            }
            telemetry.addData("Turret ticks", "%d   (start position should be %d)",
                    aimer.getTurretTicks(), Aimer.TURRET_AUTON_START_TICKS);
            telemetry.update();
        }

        waitForStart();

        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();

        launcher.setTargetVelocity(Launcher.SHOOT_VELOCITY);

        drive.driveTo(SHOOT1_X, SHOOT1_Y, SHOOT1_HEADING, FAST_DRIVE_POWER);
        drive.showPose();

        // Swing the turret from its start position (90 degrees to the side) around toward the
        // hive so the Limelight can see it, then fine-tune with auto-aim.
        turnTurretTo(SHOOT1_TURRET_TICKS, TURRET_TURN_TIMEOUT);

        // Shoot even if aiming fails: the turret is probably close enough.
        aimFor(AIM_TIMEOUT);
        sleep(2000);
        shoot(LAUNCHER_TIMEOUT);

        // get ready to scoop, intake running
        intake.setPower(INTAKE_POWER);
        drive.driveTo(GARDEN_START_X, GARDEN_START_Y, GARDEN_START_HEADING, FAST_DRIVE_POWER);
        drive.showPose();

        // scoop!
        drive.driveTo(GARDEN_END_X, GARDEN_END_Y, GARDEN_END_HEADING, SLOW_DRIVE_POWER, 0.5, 3.0 );
        drive.showPose();

        // drive to waypoint
        drive.driveTo(WAYPOINT1_X, WAYPOINT1_Y, WAYPOINT1_HEADING, FAST_DRIVE_POWER);
        drive.showPose();

        // Keep the intake running on the way so balls finish coming in.
        drive.driveTo(SHOOT2_X, SHOOT2_Y, SHOOT2_HEADING, FAST_DRIVE_POWER, 0.8, 8.0);
        drive.showPose();
        intake.setPower(0);

        // Only shoot if we can aim at the hive. If we missed shot 1 the hive won't tip, so
        // there's no point waiting longer than AIM_TIMEOUT; if it did tip, it should be done
        // by the time we get back from the garden.
        if (aimFor(AIM_TIMEOUT)) {
            shoot(LAUNCHER_TIMEOUT);
        }

        // park
        drive.driveTo(PARK_LEFT_X, PARK_LEFT_Y, PARK_LEFT_HEADING, FAST_DRIVE_POWER);

        // Leave the turret facing straight forward: TeleOp zeroes the turret wherever it is.
        turnTurretTo(0, TURRET_TURN_TIMEOUT);
        sleep(1000);

        drive.showPose();
        sleep(3000); // don't need this prolly
    }
}
