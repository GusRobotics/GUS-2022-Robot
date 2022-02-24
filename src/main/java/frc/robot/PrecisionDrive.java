package frc.robot;

// Default resources
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Rev Resources
import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private CANSparkMax m_drive_left;
    private CANSparkMax m_drive_right;
    private double set_point;
    private double rev_to_dist;

    // Tunable PID constants (kIz and kFF are additional options)
    private double kP = 0.06;
    private double kI = 0;
    private double kD = 0;
    private double allowedError = 0.025;

    // Constant PID constants (kMinOutput and kMaxOutput are additional options)
    private double correctTime = 0.125;

    // Global storage
    private double last_time;
    private double integral;
    private double last_error;
    private double first_correct;
    private boolean correct;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right) {
        // Store motors
        m_drive_left = m_left;
        m_drive_right = m_right;

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

        // kp = 0.06 on floor for long distance
        kP = 0.2;
        kI = 0;
        kD = 0;
        allowedError = 0.125;
    }

    /**
     * PID Control for driving in a straight line with motors controlled seperatedly to combat drift
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - true if the loop is done for both motors, false if it is not
     */
    public boolean pidStraight() {
        boolean done_left = pidControl(m_drive_left);
        boolean done_right = pidControl(m_drive_right);

        return (done_left && done_right);
    }

    /**
     * Iterative function that adjusts individual motor power to move it to a set point.
     * @param motor - the motor that is moved
     * @param val - the target value (either an encoder count or gyro value)
     * @return - completion status (true = done, false = in progress)
     */
    public boolean pidControl(CANSparkMax motor) {
        double error = set_point - motor.getEncoder().getPosition()*config.rev_feet_conversion;
        double dt = Timer.getFPGATimestamp() - last_time;

        // Integral (potentially cap if too large or reset on passing set point)
        integral += error * dt;

        // Derivative
        double derivative = (error - last_error)/dt;

        // Output
        double outputSpeed = error * kP + integral * kI + derivative * kD;

        // Set motors to output
        motor.set(outputSpeed);

        // Update variables
        last_time = Timer.getFPGATimestamp();
        last_error = error;
        
        // Wait a set time
        Timer.delay(0.005);

        // Print data to shuffleboard (graphing would be great)
        SmartDashboard.putNumber("value", motor.getEncoder().getPosition()*config.rev_feet_conversion);
        SmartDashboard.putNumber("target", set_point);
        SmartDashboard.putNumber("power", outputSpeed);
        SmartDashboard.putNumber("error", error);

        // Check if the system is in the correct range
        if(Math.abs(error) < allowedError) {
            if(correct) {
                // Wait until stable
                if(Timer.getFPGATimestamp() - first_correct >= correctTime) {
                    return true;
                }
            }
            else {
                // Just entered region - start count to determine stability
                correct = true;
                first_correct = Timer.getFPGATimestamp();
            }
        }
        else {
            correct = false;
        }
        return false;
    }

    
}
