package frc.robot.subsystems.shooter;

public enum ShooterState {
    LOADING_NOTE, //taken through intake
    HOLDING_NOTE, //in the conveyer belt
    NO_NOTE, //no note in robot
    FIRING, 
    VERTICAL //for start of the match
}