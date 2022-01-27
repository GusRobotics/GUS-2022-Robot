package frc.robot;

// Neo Resources
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class PrecisionDrive {
    private CANSparkMax m_left_drive;
    private CANSparkMax m_right_drive;
    private SparkMaxPIDController m_pidController;
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

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right, double conversion) {
        // Store reference to motors
        m_left_drive = m_left;
        m_right_drive = m_right;

        // Set all of the parameters for the PID loop
        m_pidController = m_left_drive.getPIDController();

        // Set conversion factor
        rev_to_dist = conversion;

        // Set all PID Coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // Start the set point to zero
        set_point_rev = 0;
    }

    /** Reset PID with new set point. Current position is zeroed
     * @param distance in feet
    */
    public void setSetPoint(double distance) {
        // Set new set point (current [rev] + distance [dist]/([dist]/[rev]) = [rev])
        // This effectively adds the distance in the absence of a proper encoder reset function
        set_point_rev = m_left_drive.getEncoder().getPosition() + distance/rev_to_dist;

    }

    /** Iterative function that adjusts power to get the drive train to its set point*/
    public void positionControl() {
        m_pidController.setReference(set_point_rev, CANSparkMax.ControlType.kPosition);
    }

    
}
