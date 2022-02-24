package frc.robot;

// Rev Resources
import com.revrobotics.CANSparkMax;

public class Climber {
    private final CANSparkMax m_hook_left;
    private final CANSparkMax m_hook_right;

    public Climber(CANSparkMax hook_left, CANSparkMax hook_right) {
        m_hook_left = hook_left;
        m_hook_right = hook_right;
    }

    /**
     * Iterative model for full hook raise when reaching for bar
     * @return boolean status completion
     */
    public boolean fullHookRaise() {
        return true;
    }

    /**
     * Iterative model for partial hook raise from lifting the variable hooks
     * @return boolean status completion
     */
    public boolean partialHookRaise() {
        return true;
    }

    // Implement five operations - full hook raise, full hook retract, partial hook raise, pneumatic extend, pneumatic retract
}
