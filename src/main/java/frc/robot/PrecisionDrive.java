package frc.robot;

import com.revrobotics.CANSparkMax;

public class PrecisionDrive {
    private DistancePID leftDistPID;
    private DistancePID rightDistPID;
    private TurnPID leftTurnPID;
    private TurnPID rightTurnPID;


    public PrecisionDrive(CANSparkMax m_left, CANSparkMax m_right) {
        // Create controllers
        leftDistPID = new DistancePID(m_left);
        rightDistPID = new DistancePID(m_right);
    }

    /**
     * Sets the distance to drive for both motors
     * @param distance - the straight distance for the robot to drive
     */
    public void setDistance(double distance) {
        leftDistPID.setSetPoint(distance);
        rightDistPID.setSetPoint(distance);
    }

    /**
     * PID Control for driving in a straight line with motors controlled seperatedly to combat drift
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - true if the loop is done for both motors, false if it is not
     */
    public boolean pidStraight() {
        boolean done_left = leftDistPID.pidControl("left_drive");
        boolean done_right = rightDistPID.pidControl("right_drive");

        return (done_left && done_right);
    }

    /**
     * Sets the angle to rotate the robot in place
     * @param angle - the angle for the robot to rotate [-180, 180]
     */
    public void setAngle(double angle) {
        leftTurnPID.setSetPoint(angle);
        leftTurnPID.setSetPoint(angle);
    }

    /**
     * PID Control for turning in place based on gyro values
     * @param leftDrive - left motor
     * @param rightDrive - right motor
     * @return - true if the loop is done for both motors, false if it is not
     */
    public boolean pidTurn() {
        boolean done_left = leftTurnPID.pidControl(true, "left_drive");
        boolean done_right = rightTurnPID.pidControl(false, "right_drive");

        return (done_left && done_right);
    }
}
