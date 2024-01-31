package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

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
    public void execute(){
        this.elevatorSubsystem.setTargetState(ElevatorState.AMP);
    }

    @Override
    public boolean isFinished(){
        if(this.elevatorSubsystem.getState()==ElevatorState.AMP){
            return true;
        }
        else return false;
    }
}