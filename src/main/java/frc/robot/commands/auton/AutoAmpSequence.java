package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.commands.sequences.PrepareAmpSequence;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Automatically runs an amp cycle including preparing the note, going to and aligning with the amp, and depositing the
 *  note (given that the robot already has a note). */
public class AutoAmpSequence extends SequentialCommandGroup {
    
    /** Constructs a new {@link AutoAmpSequence}. */
    public AutoAmpSequence(FieldManagementSubsystem fms,
                           SwerveSubsystem swerveSubsystem,
                           ElevatorSubsystem elevatorSubsystem,
                           IntakePivotSubsystem intakePivotSubsystem,
                           IntakeRollerSubsystem intakeRollerSubsystem) {

        addCommands(
            Commands.parallel(
                new PrepareAmpSequence(elevatorSubsystem, intakePivotSubsystem, intakeRollerSubsystem),
                AlignCommand.getAmpAlignCommand(swerveSubsystem, fms.isRedAlliance())
            ),
            new IntakeRollerOuttakeCommand(intakeRollerSubsystem).withTimeout(1)
        );
    }
}
