package org.firstinspires.ftc.teamcode.auton;

public interface Locations {

    // Start 1: S1 autons
    public static final double START1_X = 63;
    public static final double START1_Y = -18;
    public static final double START1_HEADING = 180;

    // Start 2: S2 autons (x, y from Auton2)
    public static final double START2_X = -63.5;
    public static final double START2_Y = -16;
    public static final double START2_HEADING = 0;

    public static final double SHOOT1_X = 57.3; // 2" off the wall so the shooter doesn't rub
    public static final double SHOOT1_Y = -18;
    public static final double SHOOT1_HEADING = 180;

    public static final double GARDEN_START_X = 50.6;
    public static final double GARDEN_START_Y = -64.6;
    public static final double GARDEN_START_HEADING = 0;

    public static final double GARDEN_END_X = 60;
    public static final double GARDEN_END_Y = -64.6;
    public static final double GARDEN_END_HEADING = 0;

    public static final double WAYPOINT1_X = -14.0;
    public static final double WAYPOINT1_Y = -37.0;
    public static final double WAYPOINT1_HEADING = 0;

    public static final double SHOOT2_X = -57;
    public static final double SHOOT2_Y = -16;
    public static final double SHOOT2_HEADING = 0;

    // Left flower: harvest by driving from START to END.
    // TODO: PLACEHOLDERS -- fill in from the notebook. Until then they equal SHOOT2, so the
    // robot just stays put instead of driving somewhere random.
    public static final double LEFT_FLOWER_START_X = -45.0;
    public static final double LEFT_FLOWER_START_Y = -28.0;
    public static final double LEFT_FLOWER_START_HEADING = 180;

    public static final double LEFT_FLOWER_END_X = -57.0;
    public static final double LEFT_FLOWER_END_Y = -28;
    public static final double LEFT_FLOWER_END_HEADING = 180;

    // TODO: PLACEHOLDER -- fill in from the notebook (these are Auton2's old park spot)
    public static final double PARK_LEFT_X = -52;
    public static final double PARK_LEFT_Y = -61;
    public static final double PARK_LEFT_HEADING = 90;

    public static final double PARK_RIGHT_X = -24;
    public static final double PARK_RIGHT_Y = -59.6;
    public static final double PARK_RIGHT_HEADING = 0;

}
