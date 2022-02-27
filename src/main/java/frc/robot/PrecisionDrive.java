package frc.robot;

import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private DistancePID leftPID;
    private DistancePID rightPID;

    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right) {
        // Create controllers
        leftPID = new DistancePID(m_left);
        rightPID = new DistancePID(m_right);
    }

    /**
     * Sets the distance to drive for both motors
     * @param distance - the straight distance for the robot to drive
     */
    public void setDistance(double distance) {
        leftPID.setSetPoint(distance);
        rightPID.setSetPoint(distance);
    }

    /**
     * PID Control for driving in a straight line with motors controlled seperatedly to combat drift
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - true if the loop is done for both motors, false if it is not
     */
    public boolean pidStraight() {
        boolean done_left = leftPID.pidControl("left_drive");
        boolean done_right = rightPID.pidControl("right_drive");

        return (done_left && done_right);
    }
}
