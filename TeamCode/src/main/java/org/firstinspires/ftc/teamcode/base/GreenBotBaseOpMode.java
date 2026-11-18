package org.firstinspires.ftc.teamcode.base;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.Aimer;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.IndicatorLight;
import org.firstinspires.ftc.teamcode.robot.Launcher;

public abstract class GreenBotBaseOpMode extends LinearOpMode {

    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected DcMotorEx intake, turret, launcherMotor, feeder;
    protected Servo wedgeServo;

    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected Drive drive;
    protected Aimer aimer;
    protected Launcher launcher;
    protected IndicatorLight aimerLed, launcherLed;

    protected double WEDGE_START = 0.7;
    protected double WEDGE_UP = 0.65;
    protected double WEDGE_DOWN = 0.25;

    // Feeder pattern for shooting in auton (see shoot()): run, back up briefly, run again.
    protected static final long FEED_FIRST_MS = 2000;
    protected static final long FEED_REVERSE_MS = 500;
    protected static final long FEED_SECOND_MS = 2000;

    /**
     * Configures the hardware that all OpModes using this robot share.
     * Call this first thing on startup.
     */
    public void config() {
        // Instantiate hardware
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        backLeft = hardwareMap.dcMotor.get("back_left");
        backRight = hardwareMap.dcMotor.get("back_right");
        intake = hardwareMap.get(DcMotorEx.class, "intake_motor");
        turret = hardwareMap.get(DcMotorEx.class, "turret_motor");
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder_motor");
        wedgeServo = hardwareMap.get(Servo.class, "wedge_servo");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        aimerLed = new IndicatorLight(hardwareMap.get(Servo.class, "aimer_led"));
        launcherLed = new IndicatorLight(hardwareMap.get(Servo.class, "launcher_led"));

        // Configure hardware
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(BRAKE);
        frontLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        pinpoint.setOffsets(-2.44, -6.56, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        drive = new Drive(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                pinpoint,
                telemetry
        );

        // Does NOT zero the turret encoder: each OpMode decides when the turret's position
        // should count as 0 (call aimer.resetTurretEncoder()).
        aimer = new Aimer(turret, limelight, telemetry);
        launcher = new Launcher(launcherMotor, telemetry);
    }

    /**
     * Set both LEDs from what the aimer and launcher know right now.
     * Call once per loop, after aimer.update().
     *
     *   Aimer LED:    pink = no hive seen, yellow = hive seen but not lined up, green = lined up
     *   Launcher LED: off = launcher stopped, pink = spinning up / recovering, green = at speed
     *
     * (Red and blue are saved for showing the alliance during INIT.)
     */
    protected void updateLights() {
        if (aimer.isAimed()) aimerLed.setColor(IndicatorLight.GREEN);
        else if (aimer.hasTarget()) aimerLed.setColor(IndicatorLight.YELLOW);
        else aimerLed.setColor(IndicatorLight.PINK);

        if (!launcher.isRunning()) launcherLed.off();
        else if (launcher.isAtSpeed()) launcherLed.setColor(IndicatorLight.GREEN);
        else launcherLed.setColor(IndicatorLight.PINK);
    }

    /**
     * Call once per INIT loop, before telemetry.update(). Either driver picks the alliance
     * (X = BLUE, B = RED; it decides which hive tags the aimer looks for), both LEDs show the
     * alliance color, and the choice is added to telemetry.
     */
    protected void selectAlliance() {
        if (gamepad1.x || gamepad2.x) aimer.setBlueAlliance(true);
        if (gamepad1.b || gamepad2.b) aimer.setBlueAlliance(false);
        showAllianceLights();
        telemetry.addData("Alliance", "%s   (X = BLUE, B = RED)",
                aimer.isBlueAlliance() ? "BLUE" : "RED");
    }

    /** Set both LEDs to the alliance color (red or blue). */
    protected void showAllianceLights() {
        double color = aimer.isBlueAlliance() ? IndicatorLight.BLUE : IndicatorLight.RED;
        aimerLed.setColor(color);
        launcherLed.setColor(color);
    }

    /**
     * For autons: wait until the launcher reaches its target velocity, keeping the LEDs and
     * telemetry current while waiting. Start the launcher first with launcher.setTargetVelocity().
     *
     * @return true if it got up to speed, false if it timed out, wasn't running, or STOP was pressed
     */
    protected boolean waitForLauncher(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && launcher.isRunning() && timer.seconds() < timeoutSeconds) {
            aimer.update();
            updateLights();
            if (launcher.isAtSpeed()) return true;
            launcher.showStatus();
            telemetry.update();
        }
        return false;
    }

    /**
     * For autons: aim the turret at the hive until it has stayed on target for
     * Aimer.AIM_SETTLE_TIME, keeping the LEDs and telemetry current while aiming. The turret is
     * stopped (braked where it is) when this returns, either way.
     *
     * Waiting for it to stay on target matters because aim() says "on target" the moment tx is
     * inside the tolerance, even if the turret is still coasting through.
     *
     * @return true if the turret is aimed, false if it timed out (no hive in view, stuck at a
     *         soft limit, or never settled) or STOP was pressed
     */
    protected boolean aimFor(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double onTargetSince = -1; // when the turret last came on target; -1 = not on target
        boolean aimed = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            aimer.update();
            boolean onTarget = aimer.aim();
            updateLights();
            aimer.showStatus();
            launcher.showStatus();
            telemetry.update();

            // make sure that the turret has been on target for a few ms, to avoid bad readings
            if (!onTarget) {
                onTargetSince = -1;                 // drifted off: start the settle wait over
            } else if (onTargetSince < 0) {
                onTargetSince = timer.seconds();    // just came on target
            } else if (timer.seconds() - onTargetSince >= Aimer.AIM_SETTLE_TIME) {
                aimed = true;                       // stayed on target long enough
                break;
            }
        }

        aimer.stopTurret();
        return aimed;
    }

    /**
     * For autons: wait for the launcher to get up to speed, then shoot: run the feeder for
     * FEED_FIRST_MS, reverse it for FEED_REVERSE_MS, then run it again for FEED_SECOND_MS.
     * Start the launcher first with launcher.setTargetVelocity(Launcher.SHOOT_VELOCITY).
     * The launcher's velocity stays on telemetry the whole time, so you can watch it drop and
     * recover as each ball goes through.
     */
    protected void shoot(double launcherTimeoutSeconds) {
        waitForLauncher(launcherTimeoutSeconds);
        runFeeder(1.0, FEED_FIRST_MS);     // shoot
        runFeeder(-1.0, FEED_REVERSE_MS);  // back up
        runFeeder(1.0, FEED_SECOND_MS);    // shoot the rest
        feeder.setPower(0);                // feeder off
    }

    /** Run the feeder at this power for this long, keeping the LEDs and telemetry current. */
    private void runFeeder(double power, long milliseconds) {
        feeder.setPower(power);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            updateLights();
            launcher.showStatus();
            telemetry.update();
        }
    }

    /**
     * For autons: turn the turret to a position in ticks from straight forward (0), keeping the
     * LEDs and telemetry current while it turns. The turret is stopped (braked where it is) when
     * this returns, either way.
     *
     * @return true if it got there, false if it timed out (e.g. blocked by a soft limit) or
     *         STOP was pressed
     */
    protected boolean turnTurretTo(int targetTicks, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        boolean there = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            aimer.update();
            there = aimer.turnTurretTo(targetTicks);
            updateLights();
            aimer.showStatus();
            telemetry.update();
            if (there) break;
        }

        aimer.stopTurret();
        return there;
    }

}
