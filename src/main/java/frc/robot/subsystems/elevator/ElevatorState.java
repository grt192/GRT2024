package frc.robot.subsystems.elevator;


import frc.robot.Constants;

/**
    States of the elevator.
*/
public enum ElevatorState {
    GROUND(Constants.ElevatorConstants.GROUND_POSITION),
    AMP(Constants.ElevatorConstants.AMP_POSITION),
    CHUTE(Constants.ElevatorConstants.CHUTE_POSITION),
    TRAP(Constants.ElevatorConstants.TRAP_POSITION),
    START(Constants.ElevatorConstants.START_POSITION);

    private final double extendDistanceMeters;

    private ElevatorState(double extendDistanceMeters) {
        this.extendDistanceMeters = extendDistanceMeters;
    }

    /**
     * get the distance of a state in meters.

     * @return meters in double.
     */
    public double getExtendDistanceMeters() {
        return this.extendDistanceMeters;
    }

}
