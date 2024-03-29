package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to position zero. */
public class ElevatorToZeroCommand extends SequentialCommandGroup{
    private ElevatorSubsystem elevatorSubsystem;
    
    /** Constructs a {@link ElevatorToZeroCommand} for the specified elevator. */
    public ElevatorToZeroCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        addCommands(
            new ElevatorToIntakeCommand(elevatorSubsystem),
            new ElevatorToLimitSwitchCommand(elevatorSubsystem)
        );
    }

     
}