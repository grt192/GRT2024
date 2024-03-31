package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to its ground intake position. */
public class ElevatorToEncoderZeroCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    /** Constructs a {@link ElevatorToEncoderZeroCommand} for the specified elevator. */
    public ElevatorToEncoderZeroCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetState(ElevatorState.ZERO);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.ZERO);
    }
}