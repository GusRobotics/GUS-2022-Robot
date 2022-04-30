package frc.robot;

public class config {
    // All measurements are in feet for distance and degrees for angle by default

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
    public static final int climber_right_ID = 12;
    public static final int climber_left_ID = 13;
    public static final int pcm_ID = 18;
    public static final int pigeon_ID = 19;

    // PWM Ports
    public static final int led_port = 0;

    // Solenoid channels
    public static final int drive_channel = 0;
    public static final int intake_channel = 1;
    public static final int climber_channel = 2;

    // Analog channels
    public static final int index_dist_sensor_channel = 0;

    // Current limit
    public static final int drive_current_limit = 50;
    public static final int intake_current_limit = 25;
    public static final int shooter_current_limit = 50;
    public static final int index_current_limit = 25;
    public static final int climber_current_limit = 25;

    // Constant Robot Stats
    public static final double rev_feet_conversion = 10/42.35;
    public static final int dist1_threshold = 375;

    // Index power
    public static final double index_power = 0.65;
    public static final double high_shot_index_power = 0.4;

    // Shot power
    public static final double low_shot_power = 0.45;

    public static final double high_shot_power = 0.7;
    public static final double high_shot_far_power = 0.9;

    // Field constants (in feet)
    public static final double high_goal_tape_height = 103/12.0;
    public static final double practice_tape_height = 72.5/12.0;

    // Robot physical constants
    public static final double camera_angle = 18.5; // 18.5?
    public static final double camera_height = 30.5/12.0;

    // Lining up to shoot
    public static final double shot_horizontal_angle_range = 2;

    // Start positions
    public static final String hangerSide = "Hangar Tarmac";
    public static final String wallSide = "Side Tarmac";

    // Climber hook encoder positions (absolute zero is the initial position) - these are not tuned
    public static final double hooks_low = 5;
    public static final double hooks_medium = 10;
    public static final double hooks_high = 15;

    public static final double left_hook_high = 200;
    public static final double right_hook_high = 200;

    public static final double controller_threshold = 0.10;

    // High gear is false

    // Colors
    public static final double pink = 0.57;
    public static final double red = 0.61;
    public static final double orange = 0.66;
    public static final double yellow = 0.69;
    public static final double green = 0.73;
    public static final double blue = 0.87;
    public static final double black = 0.99;
}
