package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnPID {
    // Motor information
    private CANSparkMax motor;
    private PigeonIMU gyro;
    private double set_point;

    // PID Constants
    private double kP = 0.0005;
    private double kI = 0;
    private double kD = 0;
    private double integral_range = 20;
    private double max_integral = 1;
    private double allowed_error = 0.15;
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

    public TurnPID(CANSparkMax motor, PigeonIMU gyro) {
        this.motor = motor;
        this.gyro = gyro;
    }

    public void setSetPoint(double angle) {
        this.set_point = this.gyro.getYaw() + angle;
    }

    public boolean pidControl(boolean invert, String id) {
        this.error = this.set_point - this.gyro.getYaw();
        this.dt = Timer.getFPGATimestamp() - this.last_time;

        // Reverse if dictated
        if(invert) {
            this.error *= -1;
        }

        // Integral (potentially cap if too large or reset on passing set point)
        if(Math.abs(this.error) < integral_range) {
            this.integral += this.error * this.dt;
        }

        if(Math.abs(this.integral) > this.max_integral) {
            this.integral = this.max_integral * (Math.abs(this.integral) / this.integral);
        }

        // Derivative
        this.derivative = (this.error - this.last_error)/this.dt;

        // Output
        this.output = this.error * this.kP + this.integral * this.kI + this.derivative * this.kD;

        // Set motors to output
        //this.motor.set(this.output);

        // Update variables
        this.last_time = Timer.getFPGATimestamp();
        this.last_error = error;
        
        // Wait a set time
        Timer.delay(0.005);

        // Print data to shuffleboard (graphing would be great)
        SmartDashboard.putNumber((id + " power"), this.output);
        SmartDashboard.putNumber((id + " error"), this.error);
        SmartDashboard.putNumber((id + " integral"), this.integral);

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