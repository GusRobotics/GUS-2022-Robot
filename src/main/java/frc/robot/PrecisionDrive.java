package frc.robot;

// Neo Resources
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class PrecisionDrive {
    private CANSparkMax m_left_drive;
    private CANSparkMax m_right_drive;
    private SparkMaxPIDController m_pid_controller;
    private double set_point;

    // PID constants
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right) {
        // Store reference to motors
        m_left_drive = m_left;
        m_right_drive = m_right;

        // Set all of the parameters for the PID loop
        m_pid_controller = m_left.getPIDController();

        // Set all PID Constants

        // Start the set point to zero
        set_point = 0;
    }

    /** Set the distance to the next point and reset motor encoders
     * @param distance in feet
    */
    public void setSetPoint(double distance) {
        set_point = distance;
    }

    /** Iterative function that adjusts power to get the drive train to its set point*/
    public void positionControl() {
        
    }

    
}
