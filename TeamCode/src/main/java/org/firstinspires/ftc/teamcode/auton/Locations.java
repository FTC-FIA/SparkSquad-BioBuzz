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

    public static final double SHOOT1_X = 61; // 2" off the wall so the shooter doesn't rub
    public static final double SHOOT1_Y = START1_Y;
    public static final double SHOOT1_HEADING = START1_HEADING;

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

    // Left flower: harvest by driving from START to END.
    // TODO: PLACEHOLDERS -- fill in from the notebook. Until then they equal SHOOT2, so the
    // robot just stays put instead of driving somewhere random.
    public static final double LEFT_FLOWER_START_X = SHOOT2_X;
    public static final double LEFT_FLOWER_START_Y = SHOOT2_Y;
    public static final double LEFT_FLOWER_START_HEADING = SHOOT2_HEADING;

    public static final double LEFT_FLOWER_END_X = SHOOT2_X;
    public static final double LEFT_FLOWER_END_Y = SHOOT2_Y;
    public static final double LEFT_FLOWER_END_HEADING = SHOOT2_HEADING;

    // TODO: PLACEHOLDER -- fill in from the notebook (these are Auton2's old park spot)
    public static final double PARK_LEFT_X = -40;
    public static final double PARK_LEFT_Y = -57;
    public static final double PARK_LEFT_HEADING = 180;

    public static final double PARK_RIGHT_X = -30;
    public static final double PARK_RIGHT_Y = -50;
    public static final double PARK_RIGHT_HEADING = 180;

}
