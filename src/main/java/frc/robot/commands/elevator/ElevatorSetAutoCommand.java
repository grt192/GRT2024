package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sets the elevator to automatic control mode. */
public class ElevatorSetAutoCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    /** Constructs a {@link ElevatorSetAutoCommand} for the specified elevator. */
    public ElevatorSetAutoCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setAuto();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}