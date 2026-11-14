package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Spins the launcher flywheel at a target velocity and tells you when it is up to speed.
 *
 * The motor's built-in velocity control (RUN_USING_ENCODER + PIDF) does the work: once a target
 * is set, the motor keeps holding it on its own. Velocities are in encoder ticks per second.
 *
 * Typical use:
 * <pre>
 *     launcher.setTargetVelocity(1250);      // start spinning up
 *     ...
 *     if (launcher.isAtSpeed()) feed();      // only shoot once it's up to speed
 *     ...
 *     launcher.stop();
 * </pre>
 *
 * Or let the calibration table pick the velocity from how far away the hive is:
 * <pre>
 *     launcher.setTargetVelocityForDistance(aimer.getDistanceInches());
 * </pre>
 */
public class Launcher {

    // ---------------- Tuning / configuration ----------------
    public static double MIN_TARGET_VELOCITY = 200;   // ticks/s
    public static double MAX_TARGET_VELOCITY = 2000;  // ticks/s
    public static double AT_SPEED_TOLERANCE = 50;     // "at speed" if within this many ticks/s

    // Velocity PIDF for the motor controller. Adjust these for spinup and recovery.
    public static double KP = 40;
    public static double KI = 2;
    public static double KD = 0;
    public static double KF = 14.65;

    // Distance -> velocity table, from the Launcher Calibration Sheet. Distance is what
    // Aimer.getDistanceInches() reads (inches); velocity is ticks/s. List the points nearest
    // first, one velocity per distance. Between two points velocityForDistance() draws a straight
    // line; past either end it uses that end's velocity.
    // PLACEHOLDER: one flat velocity until we have calibration data.
    public static double[] CAL_DISTANCES  = {   40,   80 };
    public static double[] CAL_VELOCITIES = { 1250, 1250 };

    // ---------------- Hardware and state ----------------
    private final DcMotorEx motor;
    private final Telemetry telemetry;

    private double targetVelocity = 0;  // 0 = stopped

    public Launcher(DcMotorEx motor, Telemetry telemetry) {
        this.motor = motor;
        this.telemetry = telemetry;

        checkTable(); // a typo in the table stops the OpMode at INIT, not mid-match

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF)
        );
    }

    // ---------------- Control ----------------

    /**
     * Spin at this velocity, kept between MIN_TARGET_VELOCITY and MAX_TARGET_VELOCITY.
     * 0 or less stops the launcher.
     */
    public void setTargetVelocity(double velocity) {
        if (velocity <= 0) {
            stop();
            return;
        }
        targetVelocity = Range.clip(velocity, MIN_TARGET_VELOCITY, MAX_TARGET_VELOCITY);
        motor.setVelocity(targetVelocity);
    }

    /**
     * Spin at the velocity the calibration table gives for this distance.
     * Pass it aimer.getDistanceInches().
     */
    public void setTargetVelocityForDistance(double inches) {
        setTargetVelocity(velocityForDistance(inches));
    }

    /** Raise (positive) or lower (negative) the target velocity, e.g. from a D-pad press. */
    public void changeTargetVelocity(double change) {
        setTargetVelocity(targetVelocity + change);
    }

    /** Stop driving the flywheel and let it spin down. */
    public void stop() {
        targetVelocity = 0;
        motor.setPower(0);
    }

    // ---------------- What the launcher knows ----------------

    /** True if the launcher has a target velocity (it was started and not stopped). */
    public boolean isRunning() {
        return targetVelocity > 0;
    }

    /** True if the launcher is running and within AT_SPEED_TOLERANCE of its target. */
    public boolean isAtSpeed() {
        return isRunning() && Math.abs(getVelocity() - targetVelocity) < AT_SPEED_TOLERANCE;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** The flywheel's actual velocity right now, in ticks/s. */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * The velocity that scores from this many inches away, from the calibration table.
     *
     * Between two table points it goes the same fraction of the way between their velocities:
     * halfway from 40 in (1100) to 60 in (1300) is 50 in, so 1200. Closer than the first point or
     * farther than the last, it uses that end point's velocity rather than guessing.
     */
    public static double velocityForDistance(double inches) {
        int last = CAL_DISTANCES.length - 1;
        if (inches <= CAL_DISTANCES[0]) return CAL_VELOCITIES[0];
        if (inches >= CAL_DISTANCES[last]) return CAL_VELOCITIES[last];

        // Find the first point at or past this distance; the one before it is on the near side.
        int i = 1;
        while (CAL_DISTANCES[i] < inches) i++;

        double nearDistance = CAL_DISTANCES[i - 1], farDistance = CAL_DISTANCES[i];
        double nearVelocity = CAL_VELOCITIES[i - 1], farVelocity = CAL_VELOCITIES[i];
        double fraction = (inches - nearDistance) / (farDistance - nearDistance);
        return nearVelocity + fraction * (farVelocity - nearVelocity);
    }

    /** Check the table makes sense: matching lengths, distances nearest to farthest. */
    private static void checkTable() {
        if (CAL_DISTANCES.length == 0 || CAL_DISTANCES.length != CAL_VELOCITIES.length) {
            throw new IllegalStateException("Launcher table: CAL_DISTANCES and CAL_VELOCITIES"
                    + " need the same number of entries (at least one)");
        }
        for (int i = 1; i < CAL_DISTANCES.length; i++) {
            if (CAL_DISTANCES[i] <= CAL_DISTANCES[i - 1]) {
                throw new IllegalStateException("Launcher table: CAL_DISTANCES must go nearest"
                        + " to farthest with no repeats (problem at entry " + i + ")");
            }
        }
    }

    /** Add the launcher's state to telemetry. The caller still calls telemetry.update(). */
    public void showStatus() {
        String state;
        if (!isRunning()) state = "OFF";
        else if (isAtSpeed()) state = "AT SPEED";
        else state = "spinning up";
        telemetry.addData("Launcher", "%s   (%.0f / %.0f ticks/s)",
                state, getVelocity(), targetVelocity);
    }
}
