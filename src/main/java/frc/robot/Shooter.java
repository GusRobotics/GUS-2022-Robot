package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax m_shooter_1;
    private CANSparkMax m_shooter_2;

    public Shooter(CANSparkMax shooter_1, CANSparkMax shooter_2) {
        this.m_shooter_1 = shooter_1;
        this.m_shooter_2 = shooter_2;
    }

    /**
     * Set the power of both shooter motors
     * @param power - the power of the motor as a double from 0.0 to 1.0
     */
    public void setPower(double power) {
        this.m_shooter_1.set(power);
        this.m_shooter_2.set(power);
    }

    public void setPowerLow() {
        this.setPower(config.low_shot_power);
    }

    public void setPowerHigh() {
        this.setPower(config.high_shot_power);
    }

    /**
     * Determine the shooter power for a shot of variable distance
     * @param distance - estimated distance from target (FEET)
     */
    public void setPowerAuto(double distance) {
        // Incorporate some sort of formula here, tbd
        this.setPower(config.high_shot_power);
        SmartDashboard.putString("Warning", "Incomplete variable distance tuning");
    }

    public void stop() {
        this.setPower(0);
    }
}
