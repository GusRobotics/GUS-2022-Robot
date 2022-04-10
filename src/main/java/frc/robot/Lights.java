package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights {
    private Spark lightstrip;
    private boolean on = true;
    private double time_stamp = 0;

    public Lights() {
        lightstrip = new Spark(config.led_port);
    }

    public void setColor(double color_val) {
        lightstrip.set(color_val);
    }

    public void setBlink(double color_val) {
        if(Timer.getFPGATimestamp() - time_stamp > 0.25) {
            if(on) {
                this.setOff();
            }
            else {
                this.setColor(color_val);
            }
        }
        else {
            time_stamp = Timer.getFPGATimestamp();
            on = !on;
        } 
    }

    public void setOff() {
        lightstrip.set(config.black);
    }
}
