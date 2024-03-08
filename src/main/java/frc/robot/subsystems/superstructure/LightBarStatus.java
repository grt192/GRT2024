package frc.robot.subsystems.superstructure;

/**
 * Holds state of robot Light Bar, changing LED status to alert humans.
 */
public enum LightBarStatus {
    DORMANT,
    AUTON, 
    ENDGAME,
    INTAKING,
    HOLDING_NOTE,
    AUTO_ALIGN,
    SHOOTER_SPIN_UP;

    private LightBarStatus() {

    }
}
