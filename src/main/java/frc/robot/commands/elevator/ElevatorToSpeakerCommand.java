package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToSpeakerCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToSpeakerCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void execute(){
        this.elevatorSubsystem.setTargetState(ElevatorState.SPEAKER);
    }

    @Override
    public boolean isFinished(){
        if(this.elevatorSubsystem.getState()==ElevatorState.SPEAKER){
            return true;
        }
        else return false;
    }
}