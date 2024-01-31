package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public enum ElevatorState {
    GROUND(Constants.ElevatorConstants.GROUND_POSITION),
    SPEAKER(Constants.ElevatorConstants.SPEAKER_POSITION),
    AMP(Constants.ElevatorConstants.AMP_POSITION),
    CHUTE(Constants.ElevatorConstants.CHUTE_POSITION),
    TRAP(Constants.ElevatorConstants.TRAP_POSITION),
    START(Constants.ElevatorConstants.START_POSITION);

    private final double extendDistanceMeters;

    private ElevatorState(double extendDistanceMeters){
        this.extendDistanceMeters = extendDistanceMeters;
    }

    public double getExtension(){
        return this.extendDistanceMeters;
    }

}
