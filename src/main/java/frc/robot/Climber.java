package frc.robot;

// Rev Resources
import com.revrobotics.CANSparkMax;

public class Climber {
    private final CANSparkMax m_hook_left;
    private final CANSparkMax m_hook_right;
    private String status = "";

    public Climber(CANSparkMax hook_left, CANSparkMax hook_right) {
        m_hook_left = hook_left;
        m_hook_right = hook_right;
    }

    /**
     * Iterative model for full hook raise when reaching for bar
     */
    public void fullHookRaise() {
        if(status != "high") {
            m_hook_left.getEncoder().setPosition(0);
            m_hook_right.getEncoder().setPosition(0);
            status = "high";
        }
        if(m_hook_left.getEncoder().getPosition() < config.left_hook_high) {
            // CHECK DIRECTION
            m_hook_left.set(0.2);
        }
        else {
            m_hook_left.set(0);
        }
        if(m_hook_right.getEncoder().getPosition() < config.right_hook_high) {
            // CHECK DIRECTION
            m_hook_right.set(0.2);
        }
        else {
            m_hook_right.set(0);
        }
    }

    /**
     * Iterative model for partial hook raise from lifting the variable hooks
     * @return boolean status completion
     */
    public boolean partialHookRaise() {
        if(m_hook_left.getEncoder().getPosition() < config.hooks_medium) {
            // CHECK DIRECTION
            m_hook_left.set(0);
            return false;
        }
        if(m_hook_left.getEncoder().getPosition() < config.hooks_medium) {
            // CHECK DIRECTION
            m_hook_left.set(0);
            return false;
        }
        return true;
    }

    /**
     * Iterative model for full hook retraction when lifting the robot
     * @return boolean status completion
     */
    public boolean fullHookRetract() {
        if(m_hook_left.getEncoder().getPosition() > config.hooks_low) {
            // CHECK DIRECTION
            m_hook_left.set(0);
            return false;
        }
        return true;
    }

    public void setStatus(String new_status) {
        status = new_status;
    }
}
