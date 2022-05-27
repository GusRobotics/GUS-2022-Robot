package frc.robot;

// Rev Resources
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber {
    private final CANSparkMax m_climber_left;
    private final CANSparkMax m_climber_right;
    private String status = "";

    public Climber() {
        // Create Motors
        m_climber_left = new CANSparkMax(config.climber_left_ID, MotorType.kBrushless);
        m_climber_right = new CANSparkMax(config.climber_right_ID, MotorType.kBrushless);
        m_climber_left.restoreFactoryDefaults();
        m_climber_right.restoreFactoryDefaults();
        m_climber_left.setSmartCurrentLimit(config.climber_current_limit);
        m_climber_right.setSmartCurrentLimit(config.climber_current_limit);
        m_climber_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_climber_right.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_climber_left.getEncoder().setPosition(0);
        m_climber_right.getEncoder().setPosition(0);
    }

    public void raiseLeftClimber() {
        m_climber_left.set(0.96);
    }

    public void retractLeftClimber() {
        m_climber_left.set(-0.96);
    }

    public void stopLeftClimber() {
        m_climber_left.set(0);
    }

    public void raiseRightClimber() {
        m_climber_right.set(1);
    }

    public void retractRightClimber() {
        m_climber_right.set(-1);
    }

    public void stopRightClimber() {
        m_climber_right.set(0);
    }

    /**
     * Iterative model for full hook raise when reaching for bar
     */
    public void fullHookRaise() {
        if(status != "high") {
            m_climber_left.getEncoder().setPosition(0);
            m_climber_right.getEncoder().setPosition(0);
            status = "high";
        }
        if(m_climber_left.getEncoder().getPosition() < config.left_hook_high) {
            // CHECK DIRECTION
            m_climber_left.set(0.2);
        }
        else {
            m_climber_left.set(0);
        }
        if(m_climber_right.getEncoder().getPosition() < config.right_hook_high) {
            // CHECK DIRECTION
            m_climber_right.set(0.2);
        }
        else {
            m_climber_right.set(0);
        }
    }

    /**
     * Iterative model for partial hook raise from lifting the variable hooks
     * @return boolean status completion
     */
    public boolean partialHookRaise() {
        if(m_climber_left.getEncoder().getPosition() < config.hooks_medium) {
            // CHECK DIRECTION
            m_climber_left.set(0);
            return false;
        }
        if(m_climber_left.getEncoder().getPosition() < config.hooks_medium) {
            // CHECK DIRECTION
            m_climber_left.set(0);
            return false;
        }
        return true;
    }

    /**
     * Iterative model for full hook retraction when lifting the robot
     * @return boolean status completion
     */
    public boolean fullHookRetract() {
        if(m_climber_left.getEncoder().getPosition() > config.hooks_low) {
            // CHECK DIRECTION
            m_climber_left.set(0);
            return false;
        }
        return true;
    }
}
