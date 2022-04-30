package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

public class ColorSensor {
    private final ColorSensorV3 m_colorSensor;

    public ColorSensor() {
        m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public void printVals() {
        SmartDashboard.putNumber("Red", m_colorSensor.getRed());
        SmartDashboard.putNumber("Blue", m_colorSensor.getBlue());
        SmartDashboard.putNumber("Green", m_colorSensor.getGreen());
        
    }
}
