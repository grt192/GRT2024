package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToGroundCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorToGroundCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("TO GROUND FINISHED");
        if (interrupted){
            System.out.println("TO GROUND INTERUPTED");
        }
        return;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        elevatorSubsystem.setTargetState(ElevatorState.GROUND);
    }

    @Override
    public boolean isFinished(){
        System.out.println("TO GROUND RUNNING");
        return elevatorSubsystem.atState(ElevatorState.GROUND);
    }
}