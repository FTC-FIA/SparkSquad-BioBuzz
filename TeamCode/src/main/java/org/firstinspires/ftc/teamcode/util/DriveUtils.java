package org.firstinspires.ftc.teamcode.util;

public class DriveUtils {

    // params to pass to mecanum drive
    public static class MecanumDriveParams {
        public double forward;
        public double strafe;
        public double turn;

        public MecanumDriveParams(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    public static MecanumDriveParams toFieldRelative(double fieldX, double fieldY, double heading, double power, double turn) {
        double h = Math.toRadians(heading);
        double length = Math.hypot(fieldX, fieldY);

        double forward = 0;
        double right = 0;

        if (length > 0) {
            forward = (fieldX * Math.cos(h) + fieldY * Math.sin(h)) / length;
            right = (fieldX * Math.sin(h) - fieldY * Math.cos(h)) / length;
        }

        return new MecanumDriveParams(forward * power, right * power, turn);
    }}
