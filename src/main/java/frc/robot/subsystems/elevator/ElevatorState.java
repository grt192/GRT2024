package frc.robot.subsystems.elevator;

import frc.robot.Constants;

/** States of the elevator. */
public enum ElevatorState {
    ZERO(Constants.ElevatorConstants.ZERO_POSITION),
    AMP(Constants.ElevatorConstants.AMP_POSITION),
    TRAP(Constants.ElevatorConstants.TRAP_POSITION);

    private final double extendDistanceMeters;

    private ElevatorState(double extendDistanceMeters) {
        this.extendDistanceMeters = extendDistanceMeters;
    }

    /**
     * Gets the distance of a state in meters.
     *
     * @return meters in double.
     */
    public double getExtendDistanceMeters() {
        return this.extendDistanceMeters;
    }

}
