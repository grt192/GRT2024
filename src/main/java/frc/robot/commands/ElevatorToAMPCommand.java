package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorToAMPCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToAMPCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void initialize(){
        this.elevatorSubsystem.setTargetState(ElevatorState.AMP);
    }

    @Override
    public boolean isFinished(){
        if(this.elevatorSubsystem.atState(ElevatorState.AMP)){
            return true;
        }
        else return false;
    }
}