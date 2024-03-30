package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

public class AutoIntakeSequence extends SequentialCommandGroup {

    public AutoIntakeSequence(IntakeRollerSubsystem intakeRollerSubsystem,
                              IntakePivotSubsystem intakePivotSubsystem,
                              SwerveSubsystem swerveSubsystem,
                              NoteDetectionWrapper noteDetector,
                              BaseDriveController driveController,
                              LightBarSubsystem lightBarSubsystem) {

        addCommands(new IntakePivotSetPositionCommand(intakePivotSubsystem, 1),
                    new ParallelRaceGroup(
                        new NoteAlignCommand(swerveSubsystem, noteDetector, driveController),
                        new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
                    )
        );
    }
}
