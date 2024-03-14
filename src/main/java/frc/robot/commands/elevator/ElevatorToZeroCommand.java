package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to position zero. */
public class ElevatorToZeroCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    
    /** Constructs a {@link ElevatorToZeroCommand} for the specified elevator. */
    public ElevatorToZeroCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public void execute() {
        elevatorSubsystem.setTargetState(ElevatorState.ZERO);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atGround();
    }
}