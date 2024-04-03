package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToAmpCommand;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;

/**
 * Deploys the intake, rolls the note to its amping position, then raises the elevator to amp height while putting the
 * intake at amping angle. Given that a note is already in the robot, this sequence prepares the mechanisms to deposit
 * it into the amp.
 */
public class PrepareAmpSequence extends SequentialCommandGroup {
    
    /** Constructs a new {@link PrepareAmpSequence}. */
    public PrepareAmpSequence(ElevatorSubsystem elevatorSubsystem,
                              IntakePivotSubsystem intakePivotSubsystem,
                              IntakeRollerSubsystem intakeRollerSubsystem) {

        this.addCommands(
            new IntakePivotSetPositionCommand(intakePivotSubsystem, 1) // extend pivot
                .unless(() -> !intakeRollerSubsystem.getRockwellSensorValue()), 
            new IntakeRollerOuttakeCommand(intakeRollerSubsystem, .17, .75) // run rollers to front sensor
                .unless(intakeRollerSubsystem::getAmpSensor)
                .until(() -> intakeRollerSubsystem.getFrontSensorReached()),
            Commands.parallel(
                new IntakePivotSetPositionCommand(intakePivotSubsystem, 0.2), // angle intake for scoring
                new ElevatorToAmpCommand(elevatorSubsystem) // raise elevator
            ) // These commands can run at the same time to save time.
        );
    }
}