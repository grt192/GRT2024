package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sets the elevator to manual control mode. */
public class ElevatorSetManualCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    /** Constructs a {@link ElevatorSetManualCommand} for the specified elevator. */
    public ElevatorSetManualCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setManual();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}