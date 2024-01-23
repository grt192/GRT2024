package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants;

public enum ElevatorState {
    GROUND(Constants.ElevatorConstants.GROUND_POSITION),
    SPEAKER(Constants.ElevatorConstants.SPEAKER_POSITION),
    AMP(Constants.ElevatorConstants.AMP_POSITION),
    CHUTE(Constants.ElevatorConstants.CHUTE_POSITION),
    TRAP(Constants.ElevatorConstants.TRAP_POSITION),
    START(Constants.ElevatorConstants.START_POSITION),
    ERROR(Constants.ElevatorConstants.ERROR);

    private final double extendDistanceMeters;


    private ElevatorState(double extendDistanceMeters){
        this.extendDistanceMeters = extendDistanceMeters;
    }

    public double getExtendDistanceMeters(){
        return this.extendDistanceMeters;
    }
    public ElevatorState getStateFromString(String state){
        if(state == "GROUND"){
            return ElevatorState.GROUND;
        }
        else if(state == "SPEAKER"){
            return ElevatorState.SPEAKER;
        }
        else if(state == "AMP"){
            return ElevatorState.AMP;
        }
        else if(state == "CHUTE"){
            return ElevatorState.CHUTE;
        }
        else if(state == "TRAP"){
            return ElevatorState.TRAP;
        }
        else if(state == "START"){
            return ElevatorState.START;
        }
        else{
            return ElevatorState.ERROR;
        } 
    }
    public double getExtension(){
        return this.extendDistanceMeters;
    }
}
