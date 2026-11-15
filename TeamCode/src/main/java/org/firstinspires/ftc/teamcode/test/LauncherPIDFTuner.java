package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;
import org.firstinspires.ftc.teamcode.robot.Launcher;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Tune the launcher's velocity PIDF by trying settings and comparing how the flywheel spins up.
 *
 * During INIT (and between runs), pick P, I, D, F, the target velocity, the "at speed" band and
 * how long each run lasts. Each run starts from a stopped flywheel, jumps to the target, records
 * the velocity as fast as it can, then stops the flywheel and reports:
 *
 *   Reached band     first time the velocity got within the band (when the LED would go green)
 *   Reached target   first time the velocity got all the way to the target
 *   Overshoot        how far the first peak went past the target
 *   Oscillations     swings past the band (above or below) after first reaching the target;
 *                    the first overshoot counts if it leaves the band. 0 = never left the band.
 *   Stable           when it entered the band for the last time and stayed there to the end
 *                    (needs at least MIN_STABLE_TIME in the band at the end of the run)
 *   Last 1.0 s       average / min / max velocity at the end of the run
 *
 * Results are also written to the robot log with the tag "PIDFTune". The settings here are only
 * for this OpMode: copy the ones you like into Launcher.java (KP, KI, KD, KF).
 *
 * Gamepad 1:  D-pad up/down = pick a setting, left/right = change it,
 *             LB / RB = make the step 10x smaller / larger, A = run again (after START).
 */
@TeleOp(name = "Launcher PIDF Tuner", group = "Test")
public class LauncherPIDFTuner extends GreenBotBaseOpMode {

    // ---------------- Settings you change from the gamepad ----------------
    private static final int P = 0, I = 1, D = 2, F = 3, TARGET = 4, BAND = 5, RUN_TIME = 6;
    private static final String[] NAMES =
            {"P", "I", "D", "F", "Target", "Band +/-", "Run time (s)"};

    // Start from what Launcher.java uses now.
    private final double[] value = {
            Launcher.KP, Launcher.KI, Launcher.KD, Launcher.KF,
            Launcher.SHOOT_VELOCITY, Launcher.AT_SPEED_TOLERANCE, 4.0};
    private final double[] step = {1, 0.1, 0.1, 0.05, 10, 5, 0.5};
    private final double[] min = {0, 0, 0, 0, Launcher.MIN_TARGET_VELOCITY, 5, 1};
    private final double[] max = {200, 50, 50, 50, Launcher.MAX_TARGET_VELOCITY, 200, 15};
    private int selected = P;

    // ---------------- Test settings ----------------
    private static final double STOPPED_VELOCITY = 20;  // ticks/s; below this counts as stopped
    private static final double SPIN_DOWN_TIMEOUT = 15; // s; start anyway if it takes longer
    private static final double MIN_STABLE_TIME = 0.5;  // s in the band at the end to be "stable"
    private static final double END_WINDOW = 1.0;       // s; the "last 1.0 s" numbers
    private static final int MAX_SAMPLES = 10000;
    private static final int HISTORY_SHOWN = 5;

    // ---------------- State ----------------
    private final double[] sampleTime = new double[MAX_SAMPLES];      // s since the run started
    private final double[] sampleVelocity = new double[MAX_SAMPLES];  // ticks/s
    private int sampleCount = 0;

    private VoltageSensor battery;
    private int runNumber = 0;
    private final List<String> report = new ArrayList<>();   // full report of the last run
    private final List<String> history = new ArrayList<>();  // one line per run

    @Override
    public void runOpMode() {
        config();
        launcher.stop();
        battery = hardwareMap.voltageSensor.iterator().next();

        while (opModeInInit()) {
            editSettings();
            telemetry.addLine("LAUNCHER PIDF TUNER -- pick settings, then START runs the first test");
            showSettings();
            telemetry.update();
        }

        waitForStart();
        if (opModeIsActive()) runTest();

        while (opModeIsActive()) {
            editSettings();
            if (gamepad1.aWasPressed()) {
                runTest();
                continue;
            }
            telemetry.addLine("A = run again with these settings");
            showSettings();
            telemetry.addLine();
            for (String line : report) telemetry.addLine(line);
            showHistory();
            telemetry.update();
        }

        launcher.stop();
    }

    // ---------------- One test run ----------------

    private void runTest() {
        runNumber++;
        double target = value[TARGET];

        // 1. Let the flywheel stop, so every run starts the same way.
        launcher.stop();
        ElapsedTime spinDown = new ElapsedTime();
        double velocity = launcherMotor.getVelocity();
        while (opModeIsActive() && Math.abs(velocity) > STOPPED_VELOCITY
                && spinDown.seconds() < SPIN_DOWN_TIMEOUT) {
            telemetry.addLine(String.format(Locale.US,
                    "Run #%d: waiting for the flywheel to stop (%.0f ticks/s)", runNumber, velocity));
            telemetry.update();
            velocity = launcherMotor.getVelocity();
        }
        if (!opModeIsActive()) return;

        // 2. Load the new PIDF. Switching modes first clears what the motor controller remembers
        //    from the last run (like the built-up I term).
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(value[P], value[I], value[D], value[F]));
        double volts = battery.getVoltage();

        // 3. Jump to the target and record the velocity as fast as we can. Telemetry only every
        //    0.2 s so it doesn't slow down the recording.
        sampleCount = 0;
        double lastShown = -1;
        ElapsedTime clock = new ElapsedTime();
        launcher.setTargetVelocity(target);
        while (opModeIsActive() && clock.seconds() < value[RUN_TIME] && sampleCount < MAX_SAMPLES) {
            double t = clock.seconds();
            double v = launcherMotor.getVelocity();
            sampleTime[sampleCount] = t;
            sampleVelocity[sampleCount] = v;
            sampleCount++;

            if (t - lastShown >= 0.2) {
                lastShown = t;
                telemetry.addLine(String.format(Locale.US,
                        "Run #%d RUNNING   %.1f s   %.0f / %.0f ticks/s", runNumber, t, v, target));
                telemetry.update();
            }
        }
        launcher.stop();

        if (sampleCount >= 2) analyze(volts);
        clearButtonPresses(); // ignore buttons pressed during the run
    }

    // ---------------- Working out the results ----------------

    private void analyze(double volts) {
        int n = sampleCount;
        double target = value[TARGET];
        double band = value[BAND];
        double endTime = sampleTime[n - 1];

        // Reached band / reached target: the first sample at or above each.
        double bandTime = -1, targetTime = -1;
        int targetIndex = -1;
        double highest = sampleVelocity[0];
        for (int k = 0; k < n; k++) {
            double v = sampleVelocity[k];
            highest = Math.max(highest, v);
            if (bandTime < 0 && v >= target - band) bandTime = sampleTime[k];
            if (targetIndex < 0 && v >= target) {
                targetTime = sampleTime[k];
                targetIndex = k;
            }
        }

        // First overshoot: the highest point after reaching the target, until it drops back
        // below the target (by more than half the band, so a little noise doesn't end it early).
        double overshoot = 0;
        if (targetIndex >= 0) {
            double peak = target;
            for (int k = targetIndex; k < n; k++) {
                double v = sampleVelocity[k];
                if (v > peak) peak = v;
                if (v < target - band / 2) break;
            }
            overshoot = peak - target;
        }

        // Oscillations: after reaching the target, count each swing out of the band. After a
        // swing above, another one above only counts once the velocity has crossed back below
        // the target (by half the band), and the same the other way round. That way one real
        // swing counts once, even when noise makes it wobble in and out at the edge of the band.
        int swings = 0;
        if (targetIndex >= 0) {
            boolean canCountAbove = true, canCountBelow = true;
            for (int k = targetIndex; k < n; k++) {
                double error = sampleVelocity[k] - target;
                if (error > band && canCountAbove) {
                    swings++;
                    canCountAbove = false;
                }
                if (error < -band && canCountBelow) {
                    swings++;
                    canCountBelow = false;
                }
                if (error < -band / 2) canCountAbove = true;  // crossed back below the target
                if (error > band / 2) canCountBelow = true;   // crossed back above the target
            }
        }

        // Stable: the last time it entered the band and then stayed in it to the end.
        int lastOutside = -1;
        for (int k = n - 1; k >= 0; k--) {
            if (Math.abs(sampleVelocity[k] - target) > band) {
                lastOutside = k;
                break;
            }
        }
        double stableTime = -1;
        if (lastOutside < n - 1) {
            double enteredBand = sampleTime[lastOutside + 1];
            if (endTime - enteredBand >= MIN_STABLE_TIME) stableTime = enteredBand;
        }

        // End of the run: average, min and max over the last END_WINDOW seconds.
        double sum = 0, low = Double.MAX_VALUE, high = -Double.MAX_VALUE;
        int count = 0;
        for (int k = 0; k < n; k++) {
            if (sampleTime[k] < endTime - END_WINDOW) continue;
            double v = sampleVelocity[k];
            sum += v;
            low = Math.min(low, v);
            high = Math.max(high, v);
            count++;
        }
        double average = sum / count;
        double msPerSample = 1000 * endTime / (n - 1);

        // ---------- Build the report ----------
        report.clear();
        String gains = String.format(Locale.US, "P %s  I %s  D %s  F %s",
                fmt(value[P]), fmt(value[I]), fmt(value[D]), fmt(value[F]));
        report.add(String.format(Locale.US, "RESULTS run #%d:  %s", runNumber, gains));
        report.add(String.format(Locale.US, "   target %.0f +/- %.0f,  battery %.2f V",
                target, band, volts));
        report.add("Reached band (" + fmt(target - band) + "):  "
                + (bandTime >= 0 ? secs(bandTime) : "never"));
        report.add("Reached target (" + fmt(target) + "):  "
                + (targetTime >= 0 ? secs(targetTime)
                        : String.format(Locale.US, "never (highest %.0f)", highest)));
        report.add("First overshoot:  " + (targetIndex < 0 ? "-- (never reached target)"
                : String.format(Locale.US, "+%.0f ticks/s (%.1f%%)", overshoot, 100 * overshoot / target)));
        report.add("Oscillations:  " + (targetIndex < 0 ? "--"
                : swings + "  (swings out of the band after reaching target)"));
        report.add("Stable (stays in band):  " + (stableTime >= 0 ? secs(stableTime)
                : "NOT stable by the end of the run"));
        report.add(String.format(Locale.US, "Last %.1f s:  avg %.0f, min %.0f, max %.0f",
                END_WINDOW, average, low, high));
        report.add(String.format(Locale.US, "Samples:  %d, one every %.1f ms", n, msPerSample));

        // A nudge in the right direction for the most obvious problems.
        if (targetIndex < 0) {
            report.add("HINT: never reached the target -- raise F (or I).");
        } else if (swings >= 3) {
            report.add("HINT: keeps swinging -- lower P (or I).");
        } else if (stableTime < 0) {
            report.add("HINT: not stable yet -- try a longer run time, or see the last second below.");
        } else if (overshoot > 2 * band) {
            report.add("HINT: big overshoot -- lower P or I.");
        }
        if (high - low > 2 * band) {
            report.add("NOTE: the last second covers more than the whole band "
                    + "(still swinging, or noisy readings).");
        }

        // One line for the history list, newest first.
        history.add(0, String.format(Locale.US,
                "#%d %s | band %s, target %s, over +%.0f, osc %s, stable %s",
                runNumber, gains,
                bandTime >= 0 ? secs(bandTime) : "never",
                targetTime >= 0 ? secs(targetTime) : "never",
                overshoot,
                targetIndex >= 0 ? String.valueOf(swings) : "--",
                stableTime >= 0 ? secs(stableTime) : "NO"));

        for (String line : report) RobotLog.ii("PIDFTune", line);
    }

    // ---------------- Settings screen ----------------

    private void editSettings() {
        if (gamepad1.dpadUpWasPressed()) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (gamepad1.dpadDownWasPressed()) selected = (selected + 1) % NAMES.length;
        if (gamepad1.dpadRightWasPressed()) change(+1);
        if (gamepad1.dpadLeftWasPressed()) change(-1);
        if (gamepad1.rightBumperWasPressed()) step[selected] = Math.min(round(step[selected] * 10), 1000);
        if (gamepad1.leftBumperWasPressed()) step[selected] = Math.max(round(step[selected] / 10), 0.001);
    }

    private void change(int direction) {
        double v = round(value[selected] + direction * step[selected]);
        value[selected] = Range.clip(v, min[selected], max[selected]);
    }

    private void showSettings() {
        telemetry.addLine("D-pad up/down: pick   left/right: change   LB/RB: step /10, x10");
        for (int k = 0; k < NAMES.length; k++) {
            telemetry.addData((k == selected ? ">> " : "     ") + NAMES[k],
                    "%s     (step %s)", fmt(value[k]), fmt(step[k]));
        }
    }

    private void showHistory() {
        if (history.size() < 2) return;
        telemetry.addLine();
        telemetry.addLine("Earlier runs (newest first):");
        for (int k = 1; k < Math.min(history.size(), HISTORY_SHOWN + 1); k++) {
            telemetry.addLine(history.get(k));
        }
    }

    /** Forget any button presses made while a run was going. */
    private void clearButtonPresses() {
        gamepad1.aWasPressed();
        gamepad1.dpadUpWasPressed();
        gamepad1.dpadDownWasPressed();
        gamepad1.dpadLeftWasPressed();
        gamepad1.dpadRightWasPressed();
        gamepad1.leftBumperWasPressed();
        gamepad1.rightBumperWasPressed();
    }

    // ---------------- Formatting ----------------

    /** Round to 3 decimals so repeated steps don't drift (0.1 + 0.2 = 0.30000000000000004). */
    private static double round(double x) {
        return Math.round(x * 1000) / 1000.0;
    }

    /** A number with up to 3 decimals and no trailing zeros: 40, 2.5, 14.65. */
    private static String fmt(double x) {
        return String.format(Locale.US, "%.3f", x).replaceAll("0+$", "").replaceAll("\\.$", "");
    }

    private static String secs(double seconds) {
        return String.format(Locale.US, "%.2f s", seconds);
    }
}
