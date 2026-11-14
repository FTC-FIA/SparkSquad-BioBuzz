package org.firstinspires.ftc.teamcode.auton;

public interface Locations {

    public static final double START_X = 63;
    public static final double START_Y = -18;
    public static final double START_HEADING = 180;

    public static final double SHOOT1_X = 61; // 2" off the wall so the shooter doesn't rub
    public static final double SHOOT1_Y = START_Y;
    public static final double SHOOT1_HEADING = START_HEADING;

    public static final double GARDEN_START_X = 47;
    public static final double GARDEN_START_Y = -50;
    public static final double GARDEN_START_HEADING = 0;

    public static final double GARDEN_END_X = 62.5;
    public static final double GARDEN_END_Y = GARDEN_START_Y;
    public static final double GARDEN_END_HEADING = GARDEN_START_HEADING;

    public static final double WAYPOINT1_X = -12.0;
    public static final double WAYPOINT1_Y = -48.0;
    public static final double WAYPOINT1_HEADING = GARDEN_END_HEADING;

    public static final double SHOOT2_X = -52.5;
    public static final double SHOOT2_Y = -16;
    public static final double SHOOT2_HEADING = 0;

    public static final double PARK_RIGHT_X = -30;
    public static final double PARK_RIGHT_Y = -50;
    public static final double PARK_RIGHT_HEADING = 180;

}
