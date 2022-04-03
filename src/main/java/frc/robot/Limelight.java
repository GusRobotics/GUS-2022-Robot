package frc.robot;

import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private NetworkTable limelight;

    public Limelight(NetworkTable limelight) {
        this.limelight = limelight;
    }

    public double getTargetX() {
        return  limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getDistanceToHub() {
        double angleToTarget = (config.camera_angle * 1.2 + this.getTargetY()) * (Math.PI / 180);
        return (config.high_goal_tape_height - config.camera_height) / (Math.tan(angleToTarget)) - 1 - 19.5/12;
    }

    public boolean isAlignedToShoot() {
        return (Math.abs(this.getTargetX()) < config.shot_horizontal_angle_range);
    }

    public void setTrackerCamera() {
        this.limelight.getEntry("camMode").setNumber(0);
    }

    public void setDriverCamera() {
        this.limelight.getEntry("camMode").setNumber(1);
    }

    public void ledOn(boolean on) {
        if(on) {
            this.limelight.getEntry("ledMode").setNumber(3);
        }
        else {
            this.limelight.getEntry("ledMode").setNumber(1);
        }
    }

    public boolean hasTarget() {
        if(this.limelight.getEntry("tv").getDouble(0.0) > 0) {
            return true;
        }
        return false;
    }

}
