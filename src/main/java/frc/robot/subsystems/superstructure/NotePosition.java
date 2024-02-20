package frc.robot.subsystems.superstructure;

public enum NotePosition {
    NONE(),
    INTAKING(), //seen and picking up
    INTAKE_HOLDING(),
    TRANSFER_TO_SHOOTER(),
    TRANSFER_TO_INTAKE(),
    SHOOTER_HOLDING(),
    INTAKE_READY_TO_SHOOT(),
    SHOOTER_READY_TO_SHOOT(),
    SHOOTING(),
    AMPING();

    private NotePosition(){

    }
}
