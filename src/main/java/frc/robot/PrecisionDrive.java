package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private CANSparkMax left;
    private CANSparkMax right;
    private DistancePID leftDistPID;
    private DistancePID rightDistPID;
    private TurnPID leftTurnPID;
    private TurnPID rightTurnPID;

    /**
     * Sets up a precision drive object to store data for pid control
     * @param m_left - left drive
     * @param m_right - right drive
     * @param gyro - PigeonIMU gyro sensor (for precision turning)
     */
    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right, PigeonIMU gyro) {
        left = m_left;
        right = m_right;
        leftDistPID = new DistancePID(m_left);
        rightDistPID = new DistancePID(m_right);
        leftTurnPID = new TurnPID(m_left, gyro);
        rightTurnPID = new TurnPID(m_right, gyro);
    }

    /**
     * Sets the distance to drive for both motors
     * @param distance - distance in feet to drive
     */
    public void setDistance(double distance) {
        leftDistPID.setSetPoint(distance);
        rightDistPID.setSetPoint(distance);
    }

    public void setAllowedError(double error) {
        leftDistPID.setAllowedError(error);
        rightDistPID.setAllowedError(error);
    }

    /**
     * PID Control for driving in a straight line with motors controlled seperatedly (combats drift)
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - returns true when both sides of the drive have gone the desired distance
     */
    public boolean pidStraight() {
        boolean done_left = leftDistPID.pidControl("left_drive");
        boolean done_right = rightDistPID.pidControl("right_drive");

        return (done_left && done_right);
    }

    /**
     * Sets the angle to rotate the robot in place
     * @param angle - the angle for the robot to rotate (should be between -180 and 180)
     */
    public void setAngle(double angle) {
        leftTurnPID.setSetPoint(angle);
        rightTurnPID.setSetPoint(angle);
    }

    /**
     * PID Control for turning in place based on gyro values
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - returns true when turning is done
     */
    public boolean pidTurn() {
        boolean done_left = leftTurnPID.pidControl(false, "left_drive");
        boolean done_right = rightTurnPID.pidControl(true, "right_drive");

        return (done_left && done_right);
    }

    public void stop() {
        left.set(0);
        right.set(0);
    }
}