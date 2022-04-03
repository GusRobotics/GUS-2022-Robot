package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights {
    private Spark lightstrip;
    boolean blink = false;

    public Lights() {
        lightstrip = new Spark(config.led_port);
    }

    public void setColor(double color_val) {
        lightstrip.set(color_val);
    }

    public void setOrange() {
        lightstrip.set(0.65);
    }

    public void setRed() {
        lightstrip.set(0.61);
    }

    public void setGreen() {
        lightstrip.set(0.73);
    }

    public void setBlue() {
        lightstrip.set(config.blue);
    }

    public void setBlack() {
        lightstrip.set(0.99);
    }

    public void setBlink() {
        blink = true;
    }

    public void setSolid() {
        blink = false;
    }

}
