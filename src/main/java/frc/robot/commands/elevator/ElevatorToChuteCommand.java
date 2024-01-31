package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToChuteCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToChuteCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void execute(){
        this.elevatorSubsystem.setTargetState(ElevatorState.CHUTE);
    }

    @Override
    public boolean isFinished(){
        if(this.elevatorSubsystem.getState()==ElevatorState.CHUTE){
            return true;
        }
        else return false;
    }
}