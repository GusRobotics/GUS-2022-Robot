package frc.robot;

// Default resources
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Neo Resources
import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private CANSparkMax m_drive_left;
    private CANSparkMax m_drive_right;
    private double set_point_rev;
    private double rev_to_dist;

    // PID constants
    public double kP = 0.5;
    public double kI = 0;
    public double kD = 0;
    public double kIz = 0;
    public double kFF = 0;
    public double kMinOutput = -1;
    public double kMaxOutput = 1;

    // Allowed error in feet
    public double allowedError = 0.025;
    public double correctTime = 0.125;

    // Global storage
    double last_time;
    double integral;
    double last_error;
    double first_correct;
    boolean correct;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right, double conversion) {
        // Store reference to motors
        m_drive_left = m_left;
        m_drive_right = m_right;

        // Set conversion factor
        rev_to_dist = conversion;

        // Default instance variables
        last_time = Timer.getFPGATimestamp();
        integral = 0;
    }

    /** Reset PID with new set point. Current position is zeroed
     * @param distance in feet
    */
    public void setSetPoint(double distance) {
        // Find distance to travel in rev: [dist]/([dist]/[rev])
        double revs = distance/rev_to_dist;

        // This effectively adds the distance in the absence of a proper encoder reset function
        set_point_rev = m_drive_left.getEncoder().getPosition() + revs;

        // Reset all variables
        last_time = Timer.getFPGATimestamp();
        integral = 0;
        last_error = revs;

    }

    /** Iterative function that adjusts power to get the drive train to its set point
     * @return boolean complete that indicates if the error is within an acceptable range
    */
    public boolean positionControl() {
        double error = set_point_rev - m_drive_left.getEncoder().getPosition();

        // Find time elapsed
        double dt = Timer.getFPGATimestamp() - last_time;

        // Update integral
        integral += error * dt;

        // Potentially reset integral on passing set point, also consider capping it or reseting it if it gets too big

        // Update derivative
        double derivative = (error - last_error)/dt;

        // Determine output speed
        double outputSpeed = error * kP + integral * kI + derivative * kD;

        // Set motors to calculated speed
        m_drive_left.set(outputSpeed);
        m_drive_right.set(outputSpeed);

        // Update variables
        last_time = Timer.getFPGATimestamp();
        last_error = error;
        
        // Wait a set time
        // [DELAY]

        // Print data to shuffleboard (graphing would be great)
        SmartDashboard.putNumber("encoder value", m_drive_left.getEncoder().getPosition());
        SmartDashboard.putNumber("power", outputSpeed);
        SmartDashboard.putNumber("error", error);

        if(Math.abs(error) < allowedError/rev_to_dist) {
            if(correct) {
                // Wait until stable
                if(Timer.getFPGATimestamp() - first_correct >= correctTime) {
                    return true;
                }
                else {
                    // Indicate position is in correct range
                    correct = true;
                    first_correct = Timer.getFPGATimestamp();
                }
            }
            else {
                correct = false;
            }
            
        }
        return false;
    }

    
}
