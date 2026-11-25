package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Aimer;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Start 2 -> SHOOT2 (wait for our partner to tip the hive) -> left flower -> SHOOT2 -> park left.
 *
 * Setup: robot at START2, turret turned to its auton start position (same as the S1 auton,
 * Aimer.TURRET_AUTON_START_TICKS). Press INIT, pick the alliance, check that "Turret ticks" on
 * the Driver Station matches the start position.
 */
@Autonomous(name = "Auton 1: S2_LF_S2_PL", group = "Auton")
public class S2_LF_S2_PL extends GreenBotBaseOpMode implements Locations {

    static final double FAST_DRIVE_POWER = 1.0;
    static final double SLOW_DRIVE_POWER = 0.3;
    static final double INTAKE_POWER = 1.0;

    // Where to point the turret at SHOOT2 before auto-aiming, in ticks from straight forward.
    // 0 = straight forward; change it if the hive isn't in the Limelight's view from there.
    static final int SHOOT2_TURRET_TICKS = 0;

    static final double TURRET_TURN_TIMEOUT = 1.5; // seconds
    static final double TIP_WAIT_TIMEOUT = 25.0;   // seconds to wait for the partner to tip the hive
    static final double AIM_TIMEOUT = 1.5;         // seconds
    static final double LAUNCHER_TIMEOUT = 2.0;    // seconds

    static final double ENOUGH_TIME_TO_SHOOT = 20.0;

    @Override
    public void runOpMode() {
        config();
        drive.setUsePid(true);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START2_X, START2_Y,
                AngleUnit.DEGREES, START2_HEADING));

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

        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();

        wedgeServo.setPosition(WEDGE_UP);
        aimer.start();

        launcher.setTargetVelocity(Launcher.SHOOT_VELOCITY);

        drive.driveTo(SHOOT2_X, SHOOT2_Y, SHOOT2_HEADING, FAST_DRIVE_POWER);
        drive.showPose();

        // Swing the turret from its start position around toward the hive.
        turnTurretTo(SHOOT2_TURRET_TICKS, TURRET_TURN_TIMEOUT);

        boolean isAimed = aimFor(TIP_WAIT_TIMEOUT);
        if (isAimed) {
            sleep(1000);
            shoot(LAUNCHER_TIMEOUT);

            // if there's time, go get more pollen and shoot
            if (timer.seconds() < ENOUGH_TIME_TO_SHOOT) {

                // Harvest the left flower, intake running.
                intake.setPower(INTAKE_POWER);
                drive.driveTo(LEFT_FLOWER_START_X, LEFT_FLOWER_START_Y, LEFT_FLOWER_START_HEADING,
                        FAST_DRIVE_POWER);
                drive.showPose();

                wedgeServo.setPosition(WEDGE_DOWN); // wedge down while driving into the flower
                drive.driveTo(LEFT_FLOWER_END_X, LEFT_FLOWER_END_Y, LEFT_FLOWER_END_HEADING,
                        SLOW_DRIVE_POWER);
                drive.showPose();

                sleep(1000);

                drive.driveTo(LEFT_FLOWER_START_X, LEFT_FLOWER_START_Y, LEFT_FLOWER_START_HEADING,
                        FAST_DRIVE_POWER);
                drive.showPose();
                wedgeServo.setPosition(WEDGE_UP);

                // Back to SHOOT2. Keep the intake running on the way so balls finish coming in.
                drive.driveTo(SHOOT2_X, SHOOT2_Y, SHOOT2_HEADING, FAST_DRIVE_POWER);
                drive.showPose();
                intake.setPower(0);

                // Only shoot if we can aim at the hive (same as the S1 auton's second shot).
                if (aimFor(AIM_TIMEOUT)) {
                    shoot(LAUNCHER_TIMEOUT);
                }
            }
        }

        // park
        drive.driveTo(PARK_LEFT_X, PARK_LEFT_Y, PARK_LEFT_HEADING, FAST_DRIVE_POWER);

        // Leave the turret facing straight forward: TeleOp zeroes the turret wherever it is.
        turnTurretTo(0, TURRET_TURN_TIMEOUT);

        drive.showPose();
        telemetry.update();
    }
}
