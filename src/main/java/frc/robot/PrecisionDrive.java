package frc.robot;

// Neo Resources
import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private CANSparkMax m_left_drive;
    private CANSparkMax m_right_drive;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right) {
        m_left_drive = m_left;
        m_right_drive = m_right;
    }

    /** Drive a precise distance using PID control
     * @param distance is the distance in feet the robot will travel
     */
    public void driveDistance(double distance) {
        
    }
}
