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
 * During INIT (and between runs), pick P, I, D, F, the target velocity, the "at speed" band, how
 * long it must stay in the band to count as settled, and the longest a run may last. Each run
 * starts from a stopped flywheel and jumps to the target. It keeps going until the velocity has
 * stayed inside the band for "Stable for" seconds in a row (or "Max run" runs out), then stops the
 * flywheel and reports:
 *
 *   Reached band     first time the velocity got within the band (when the LED would go green)
 *   Reached target   first time the velocity got all the way to the target
 *   Crossings        every time the velocity crossed the target and went out of the band on
 *                    the other side, with how far past it got before turning around (the
 *                    first one is the overshoot). Wobbles that stay inside the band don't count.
 *   Settled          when it entered the band for the last time and then stayed there
 *   Last N s         average / min / max velocity at the end of the run
 *
 * Results are also written to the robot log with the tag "PIDFTune" (all crossings, even the ones
 * that don't fit on the screen). The settings here are only for this OpMode: copy the ones you
 * like into Launcher.java (KP, KI, KD, KF).
 *
 * Gamepad 1:  D-pad up/down = pick a setting, left/right = change it,
 *             LB / RB = make the step 10x smaller / larger, A = run again (after START).
 */
@TeleOp(name = "Launcher PIDF Tuner", group = "Test")
public class LauncherPIDFTuner extends GreenBotBaseOpMode {

    // ---------------- Settings you change from the gamepad ----------------
    private static final int P = 0, I = 1, D = 2, F = 3, TARGET = 4, BAND = 5, HOLD = 6, MAX_RUN = 7;
    private static final String[] NAMES =
            {"P", "I", "D", "F", "Target", "Band +/-", "Stable for (s)", "Max run (s)"};

    // Start from what Launcher.java uses now.
    private final double[] value = {
            Launcher.KP, Launcher.KI, Launcher.KD, Launcher.KF,
            Launcher.SHOOT_VELOCITY, Launcher.AT_SPEED_TOLERANCE, 2.0, 30};
    private final double[] step = {1, 0.1, 0.1, 0.05, 10, 5, 0.5, 5};
    private final double[] min = {0, 0, 0, 0, Launcher.MIN_TARGET_VELOCITY, 5, 0.5, 5};
    private final double[] max = {200, 50, 50, 50, Launcher.MAX_TARGET_VELOCITY, 200, 10, 60};
    private int selected = P;

    // ---------------- Test settings ----------------
    private static final double STOPPED_VELOCITY = 20;  // ticks/s; below this counts as stopped
    private static final double SPIN_DOWN_TIMEOUT = 15; // s; start anyway if it takes longer
    private static final int MAX_SAMPLES = 50000;       // plenty for a 60 s run
    private static final int CROSSINGS_SHOWN = 8;       // the rest go to the robot log only
    private static final int HISTORY_SHOWN = 5;

    // ---------------- State ----------------
    private final double[] sampleTime = new double[MAX_SAMPLES];      // s since the run started
    private final double[] sampleVelocity = new double[MAX_SAMPLES];  // ticks/s
    private int sampleCount = 0;

    /** One trip to one side of the target: when it crossed, and how far past it got. */
    private static class Crossing {
        final double time;   // s; when it crossed the target
        final boolean up;    // true = crossed going up (an overshoot), false = going down
        double farthest = 0; // ticks/s past the target before it turned around (+ above, - below)

        Crossing(double time, boolean up) {
            this.time = time;
            this.up = up;
        }

        void update(double error) {
            if (up ? error > farthest : error < farthest) farthest = error;
        }
    }

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
        double band = value[BAND];
        double hold = value[HOLD];

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

        // 3. Jump to the target and record the velocity as fast as we can, watching for crossings,
        //    until it has stayed in the band for `hold` seconds. Telemetry only every 0.2 s so it
        //    doesn't slow down the recording.
        sampleCount = 0;
        double bandTime = -1, targetTime = -1;
        List<Crossing> crossings = new ArrayList<>();
        Crossing current = null;     // the crossing whose swing is still going on
        double lastSignChange = 0;   // when the velocity last went from below to above the target or back
        double previousError = -1;
        double inBandSince = -1;     // when it last came into the band; -1 = outside it now
        double settledAt = -1;       // when it came into the band for the last time
        double lastShown = -1;
        ElapsedTime clock = new ElapsedTime();
        launcher.setTargetVelocity(target);

        while (opModeIsActive() && clock.seconds() < value[MAX_RUN] && sampleCount < MAX_SAMPLES) {
            double t = clock.seconds();
            double v = launcherMotor.getVelocity();
            double error = v - target;   // + = too fast, - = too slow
            sampleTime[sampleCount] = t;
            sampleVelocity[sampleCount] = v;
            sampleCount++;

            if (bandTime < 0 && error >= -band) bandTime = t;
            if ((error >= 0) != (previousError >= 0)) lastSignChange = t;
            previousError = error;

            // Crossings. The first one is when it first reaches the target. After that, a crossing
            // only counts once the velocity goes all the way out of the band on the other side, so
            // noise wobbling around the target doesn't count; its time is when it actually
            // crossed the target on the way.
            if (current == null) {
                if (error >= 0) {
                    targetTime = t;
                    current = new Crossing(t, true);
                    crossings.add(current);
                }
            } else if (current.up && error < -band) {
                current = new Crossing(lastSignChange, false);
                crossings.add(current);
            } else if (!current.up && error > band) {
                current = new Crossing(lastSignChange, true);
                crossings.add(current);
            }
            if (current != null) current.update(error);

            // Settled: inside the band for `hold` seconds in a row.
            if (Math.abs(error) <= band) {
                if (inBandSince < 0) inBandSince = t;
                if (t - inBandSince >= hold) {
                    settledAt = inBandSince;
                    break;
                }
            } else {
                inBandSince = -1;
            }

            if (t - lastShown >= 0.2) {
                lastShown = t;
                telemetry.addLine(String.format(Locale.US,
                        "Run #%d RUNNING   %.1f s   %.0f / %.0f ticks/s", runNumber, t, v, target));
                telemetry.addLine(String.format(Locale.US, "   crossings so far: %d   %s",
                        crossings.size(),
                        inBandSince >= 0 ? String.format(Locale.US, "in band for %.1f s", t - inBandSince)
                                : "outside the band"));
                telemetry.update();
            }
        }
        launcher.stop();

        if (sampleCount >= 2) {
            makeReport(volts, bandTime, targetTime, crossings, settledAt,
                    sampleCount >= MAX_SAMPLES);
        }
        clearButtonPresses(); // ignore buttons pressed during the run
    }

    // ---------------- Working out the results ----------------

    private void makeReport(double volts, double bandTime, double targetTime,
                            List<Crossing> crossings, double settledAt, boolean bufferFull) {
        int n = sampleCount;
        double target = value[TARGET];
        double band = value[BAND];
        double hold = value[HOLD];
        double endTime = sampleTime[n - 1];

        double highest = sampleVelocity[0];
        for (int k = 0; k < n; k++) highest = Math.max(highest, sampleVelocity[k]);

        // End of the run: average, min and max over the last `hold` seconds.
        double sum = 0, low = Double.MAX_VALUE, high = -Double.MAX_VALUE;
        int count = 0;
        for (int k = 0; k < n; k++) {
            if (sampleTime[k] < endTime - hold) continue;
            double v = sampleVelocity[k];
            sum += v;
            low = Math.min(low, v);
            high = Math.max(high, v);
            count++;
        }
        double average = sum / count;
        double msPerSample = 1000 * endTime / (n - 1);
        double overshoot = crossings.isEmpty() ? 0 : crossings.get(0).farthest;

        // ---------- Build the report ----------
        report.clear();
        List<String> logOnly = new ArrayList<>();
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

        report.add("Crossings of the target:  " + crossings.size());
        for (int k = 0; k < crossings.size(); k++) {
            Crossing c = crossings.get(k);
            String line = String.format(Locale.US, "   %d. %s at %s, then %s %.0f",
                    k + 1, c.up ? "up  " : "down", secs(c.time),
                    c.up ? "peaked" : "dipped", c.farthest);
            if (k < CROSSINGS_SHOWN) report.add(line);
            else logOnly.add(line);
        }
        if (!logOnly.isEmpty()) {
            report.add("   ... " + logOnly.size() + " more in the robot log (PIDFTune)");
        }

        if (settledAt >= 0) {
            report.add(String.format(Locale.US, "Settled (in band %.1f s):  %s",
                    hold, secs(settledAt)));
        } else {
            report.add(String.format(Locale.US, "Settled:  NO -- %s after %.1f s",
                    bufferFull ? "ran out of sample space" : "stopped", endTime));
        }
        report.add(String.format(Locale.US, "Last %.1f s:  avg %.0f, min %.0f, max %.0f",
                hold, average, low, high));
        report.add(String.format(Locale.US, "Samples:  %d, one every %.1f ms", n, msPerSample));

        // A nudge in the right direction for the most obvious problems.
        if (targetTime < 0) {
            report.add("HINT: never reached the target -- raise F (or I).");
        } else if (crossings.size() >= 4) {
            report.add("HINT: keeps swinging -- lower I first, then P.");
        } else if (overshoot > 2 * band) {
            report.add("HINT: big overshoot -- lower I or P (or F if it overshoots with I = 0).");
        } else if (settledAt < 0) {
            report.add("HINT: never settled -- if it hovers just outside the band, the band may be"
                    + " tighter than the velocity noise.");
        }

        // One line for the history list, newest first.
        history.add(0, String.format(Locale.US,
                "#%d %s | target %s, over +%.0f, crossings %d, settled %s",
                runNumber, gains,
                targetTime >= 0 ? secs(targetTime) : "never",
                overshoot,
                crossings.size(),
                settledAt >= 0 ? secs(settledAt) : "NO"));

        for (String line : report) {
            if (line.startsWith("   ... ")) {
                for (String extra : logOnly) RobotLog.ii("PIDFTune", extra);
            } else {
                RobotLog.ii("PIDFTune", line);
            }
        }
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
