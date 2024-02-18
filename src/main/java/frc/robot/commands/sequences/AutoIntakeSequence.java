package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForwardCommand;
import frc.robot.commands.elevator.ElevatorToGroundCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

public class AutoIntakeSequence extends SequentialCommandGroup {


    public AutoIntakeSequence(
        ElevatorSubsystem elevatorSubsystem, 
        IntakeRollersSubsystem intakeRollersSubsystem,
        SwerveSubsystem swerveSubsystem,
        NoteDetectionWrapper noteDetector,
        LEDSubsystem ledSubsystem
    ){
        addCommands(new ElevatorToGroundCommand(elevatorSubsystem)
        .andThen(new NoteAlignCommand(swerveSubsystem, noteDetector))
        .andThen(new ParallelDeadlineGroup(
            new IntakeRollerIntakeCommand(intakeRollersSubsystem, ledSubsystem).withTimeout(3),
            new DriveForwardCommand(swerveSubsystem)))
        );
    }
}
