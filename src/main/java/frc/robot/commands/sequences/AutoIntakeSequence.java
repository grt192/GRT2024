package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForwardCommand;
import frc.robot.commands.elevator.ElevatorToZeroCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

/**
 * Detects a note, moves the robot to it, then intakes it.
 */
public class AutoIntakeSequence extends SequentialCommandGroup {

    /**
     * Constructs a new {@link AutoIntakeSequence}.
     *
     * @param elevatorSubsystem The subsystem to lower the intake.
     * @param intakeRollersSubsystem The subsystem to intake the note.
     * @param swerveSubsystem The subsystem to move the robot to the note.
     * @param noteDetector The note detector that will be used to identify and locate the note.
     * @param lightBarSubsystem The subsystem to display that the intake is active.
     */
    public AutoIntakeSequence(
        ElevatorSubsystem elevatorSubsystem, 
        IntakeRollersSubsystem intakeRollersSubsystem,
        SwerveSubsystem swerveSubsystem,
        NoteDetectionWrapper noteDetector,
        LightBarSubsystem lightBarSubsystem
    ){
        addCommands(new ElevatorToZeroCommand(elevatorSubsystem)
        .andThen(new NoteAlignCommand(swerveSubsystem, noteDetector))
        .andThen(new ParallelDeadlineGroup(
            new IntakeRollerIntakeCommand(intakeRollersSubsystem, lightBarSubsystem).withTimeout(3),
            new DriveForwardCommand(swerveSubsystem)))
        );
    }
}
