package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;

public class ElevatorToGroundCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToGroundCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void initialize(){
        this.elevatorSubsystem.setTargetState(ElevatorState.GROUND);
    }

    @Override
    public boolean isFinished(){
        if(this.elevatorSubsystem.atState(ElevatorState.GROUND)){
            return true;
        }
        else return false;
    }
}