package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax m_shooter_1;
    private CANSparkMax m_shooter_2;
    private double time_stamp;
    private double prev_rps;

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

    public double getVelocity() {
        // Keep speed the same unless enough time passes for a new measure
        double dr = Math.abs(m_shooter_1.getEncoder().getPosition());
        double dt = Timer.getFPGATimestamp() - time_stamp;
        double rev_per_second = dr/dt;
        double average_rps = (prev_rps + rev_per_second) / 2;

        // Reset encoder
        m_shooter_1.getEncoder().setPosition(0);

        // Reset time
        time_stamp = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("Shooter RPS", average_rps);

        prev_rps = rev_per_second;

        return average_rps;
    }

    /**
     * Determine the shooter power for a shot of variable distance
     * @param distance - estimated distance from target (FEET)
     */
    public void setPowerAuto(double distance) {
        // Incorporate some sort of formula here, tbd
        // double p = 0.00131 * distance * distance - 0.00262 * distance + 0.635;
        // Consistently a bit long --> decrease by 0.003
        double p = distance * 0.0244 + 0.452;
        this.setPower(p);
        SmartDashboard.putNumber("Power", p);
    }

    public void stop() {
        this.setPower(0);
    }
}
