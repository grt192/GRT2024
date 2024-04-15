package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

/**
 * Attempts to intake while moving towards the nearest detected note.
 * This command deploys the intake when run, but does not stow it when complete.
 */
public class AutoIntakeSequence extends SequentialCommandGroup {

    /** Constructs an {@link AutoIntakeSequence} using the specified subsystems. */
    public AutoIntakeSequence(IntakeRollerSubsystem intakeRollerSubsystem,
                              IntakePivotSubsystem intakePivotSubsystem,
                              SwerveSubsystem swerveSubsystem,
                              NoteDetectionWrapper noteDetector,
                              BaseDriveController driveController,
                              LightBarSubsystem lightBarSubsystem) {

        addCommands(
            new IntakePivotSetPositionCommand(intakePivotSubsystem, 1),
            new ParallelCommandGroup(
                new NoteAlignCommand(swerveSubsystem, noteDetector, driveController)
                    .until(intakeRollerSubsystem::getFrontSensorValue),
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
            )
        );
    }
}
