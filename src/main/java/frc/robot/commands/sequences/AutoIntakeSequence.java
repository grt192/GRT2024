package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.DriveForwardCommand;
import frc.robot.commands.elevator.ElevatorToGroundCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoIntakeSequence extends SequentialCommandGroup{


    public AutoIntakeSequence(
        ElevatorSubsystem elevatorSubsystem, 
        IntakeRollersSubsystem intakeRollersSubsystem,
        SwerveSubsystem swerveSubsystem
    ){
        addCommands(new ElevatorToGroundCommand(elevatorSubsystem)
        .andThen(new NoteAlignCommand(swerveSubsystem))
        .andThen(new ParallelDeadlineGroup(
            new IntakeRollerIntakeCommand(intakeRollersSubsystem).withTimeout(3),
            new DriveForwardCommand(swerveSubsystem)))
        );
    }
}
