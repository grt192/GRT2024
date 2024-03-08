package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to its trap-scoring position. */
public class ElevatorToTrapCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    /** Constructs a {@link ElevatorToTrapCommand} for the specified elevator. */
    public ElevatorToTrapCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetState(ElevatorState.TRAP);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.TRAP);
    }
}