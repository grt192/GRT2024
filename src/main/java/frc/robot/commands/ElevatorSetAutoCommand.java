package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorSetAutoCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    public ElevatorSetAutoCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        return;
    }

    @Override
    public void initialize(){
        this.elevatorSubsystem.setAuto();
    }

    public boolean isFinished(){
        return true;
    }
}