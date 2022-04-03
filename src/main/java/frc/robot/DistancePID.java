package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistancePID {
    // Motor information
    private CANSparkMax motor;
    private double set_point;

    // PID Constants
    private double kP = 0.07;
    private double kI = 0.092;
    private double kD = 0;
    private double integral_range = 0.5;
    private double max_integral = 1;
    private double allowed_error = 0.25;
    private double correct_time = 0.1;

    // Global variables
    private double last_time;
    private double error;
    private double last_error;
    private double integral;
    private double derivative;
    private double dt;
    private double output;
    private boolean correct;
    private double first_correct;

    public DistancePID(CANSparkMax motor) {
        this.motor = motor;
    }

    public void setSetPoint(double distance) {
        this.set_point = this.motor.getEncoder().getPosition()*config.rev_feet_conversion - distance;
        this.allowed_error = Math.max(0.15, distance * 0.03);
    }

    public void setAllowedError(double allowed_error) {
        this.allowed_error = allowed_error;
    }

    public boolean pidControl(String id) {
        this.error = this.set_point - motor.getEncoder().getPosition()*config.rev_feet_conversion;
        this.dt = Timer.getFPGATimestamp() - this.last_time;

        // Integral
        if(Math.abs(this.error) < integral_range) {
            this.integral += this.error * this.dt;
        }

        // Cap integral
        if(Math.abs(this.integral) > this.max_integral) {
            this.integral = this.max_integral * (Math.abs(this.integral) / this.integral);
        }

        // Reset integral on passing set point
        if(this.error * this.last_error < 0) {
            this.integral = 0;
        }

        // Derivative
        this.derivative = (this.error - this.last_error)/this.dt;

        // Output
        this.output = this.error * this.kP + this.integral * this.kI + this.derivative * this.kD;

        // Set motors to output
        this.motor.set(this.output);

        // Update variables
        this.last_time = Timer.getFPGATimestamp();
        this.last_error = error;
        
        // Wait a set time
        Timer.delay(0.005);

        // Check if the system is in the correct range
        if(Math.abs(error) < this.allowed_error) {
            if(correct) {
                // Wait until stable
                if(Timer.getFPGATimestamp() - this.first_correct >= this.correct_time) {
                    return true;
                }
            }
            else {
                // Just entered region - start count to determine stability
                this.correct = true;
                this.first_correct = Timer.getFPGATimestamp();
            }
        }
        else {
            this.correct = false;
        }
        return false;

    }
}