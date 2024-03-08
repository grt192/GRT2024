package frc.robot.subsystems.superstructure;

/**
 * Holds current state of match.
 */
public enum MatchStatus {
    DORMANT,
    AUTON, 
    TELEOP,
    ENDGAME;
    
    private MatchStatus() {

    }
}
