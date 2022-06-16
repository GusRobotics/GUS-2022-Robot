package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    Solenoid intakeSolenoid;

    public Intake() {   
        intakeSolenoid = new Solenoid(config.pcm_ID, PneumaticsModuleType.CTREPCM, config.intake_channel);
    }

    public void extend() {
        
    }
}
