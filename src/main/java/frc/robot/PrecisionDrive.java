package frc.robot;

// Default resources
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Rev Resources
import com.revrobotics.CANSparkMax;

// Cross the Road Resources
import com.ctre.phoenix.sensors.PigeonIMU;

public class PrecisionDrive {
    private CANSparkMax m_drive_left;
    private CANSparkMax m_drive_right;
    private double set_point;
    private double rev_to_dist;

    // Tunable PID constants
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kIz = 0;
    public double kFF = 0;
    public double allowedError = 0.025;

    // Constant PID constants
    public final double kMinOutput = -1;
    public final double kMaxOutput = 1;
    public double correctTime = 0.125;

    // Global storage
    double last_time;
    double integral;
    double last_error;
    double first_correct;
    boolean correct;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right, double conversion) {
        m_drive_left = m_left;
        m_drive_right = m_right;
        rev_to_dist = conversion;

        // Default instance variables
        last_time = Timer.getFPGATimestamp();
        integral = 0;
    }

    /** Reset PID with new set point and set to go a given distance
     * @param distance in feet
    */
    public void setDistanceControl(double distance) {
        // This effectively adds the distance in the absence of a proper encoder reset function
        set_point = m_drive_left.getEncoder().getPosition()*rev_to_dist + distance;

        // Reset all variables
        last_time = Timer.getFPGATimestamp();
        integral = 0;
        last_error = distance;

        // Distance Constants:
        kP = 0.5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0;
        allowedError = 0.125;
    }

    /** Iterative function that adjusts power to get the drive train to its set point
     * @param val is the current value of the system 
     * Distance (feet): m_drive_left.getEncoder().getPosition()*rev_to_dist (rotations --> distance)
     * Rotations (degrees): gyro value (no transformation)
     * @return boolean complete that indicates if the error is within an acceptable range
    */
    public boolean pidControl(double val) {
        double error = set_point - val;
        double dt = Timer.getFPGATimestamp() - last_time;

        // Integral (potentially cap if too large or reset on passing set point)
        integral += error * dt;

        // Derivative
        double derivative = (error - last_error)/dt;

        // Output
        double outputSpeed = error * kP + integral * kI + derivative * kD;

        // Set motors to output
        m_drive_left.set(outputSpeed);
        m_drive_right.set(outputSpeed);

        // Update variables
        last_time = Timer.getFPGATimestamp();
        last_error = error;
        
        // Wait a set time
        Timer.delay(0.005);

        // Print data to shuffleboard (graphing would be great)
        SmartDashboard.putNumber("value", val);
        SmartDashboard.putNumber("power", outputSpeed);
        SmartDashboard.putNumber("error", error);

        if(Math.abs(error) < allowedError) {
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
