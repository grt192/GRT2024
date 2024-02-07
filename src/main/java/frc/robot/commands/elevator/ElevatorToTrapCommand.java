package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToTrapCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToTrapCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void execute(){
        this.elevatorSubsystem.setTargetState(ElevatorState.TRAP);
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.atState(ElevatorState.TRAP);
    }
}