package frc.robot;

public class config {
    // CAN IDs- reserved: (RoboRio, 0), (PDB, 1)
    public static final int drive_left1_ID = 2;
    public static final int drive_left2_ID = 3;
    public static final int drive_left3_ID = 4; 
    public static final int drive_right1_ID = 5;
    public static final int drive_right2_ID = 6;
    public static final int drive_right3_ID = 7;
    public static final int shooter1_ID = 8;
    public static final int shooter2_ID = 9;
    public static final int index_ID = 10;
    public static final int intake_ID = 11;
    public static final int pcm_ID = 18;

    // Solenoid channels
    public static final int drive_channel = 0;
    public static final int intake_channel = 1;

    // Analog channels
    public static final int index_dist_sensor_channel = 0;

    // Current limit
    public static final int drive_current_limit = 50;
    public static final int intake_current_limit = 25;
    public static final int shooter_current_limit = 50;
    public static final int index_current_limit = 50;

    // Constant Robot Stats
    public static final double rev_feet_conversion = 10/42.35;
    public static final int dist1_threshold = 1000;
    public static final double low_shot_power = 0.42;
    public static final double high_shot_power = 0.72;
}