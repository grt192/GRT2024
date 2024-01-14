package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public enum ElevatorState {
    GROUND([position]),
    SPEAKER([position]),
    AMP([position]),
    CHUTE([position]),
    TRAP([position]),
    START([position]);

    private final double extendDistanceMeters;

    private ElevatorState(double extendDistanceMeters){
        this.extendDistanceMeters = extendDistanceMeters;
    }

    public double getExtension(){
        return this.extendDistanceMeters;
    }

}
