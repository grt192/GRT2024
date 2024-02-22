package frc.robot.commands.sequences;

import static frc.robot.Constants.ShooterConstants.FEED_ANGLE;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToGroundCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.shooter.feed.ShooterFeedLoadCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotSetAngleCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.NotePosition;

/**
 *  The command sequence to move the note within the robot into a shooting position and prepare the shooter to shoot.
 */
public class ShootModeSequence extends SequentialCommandGroup {

    /*
     *                    START
     *                /   |     `---------.
     *  Angle shooter    lower         Start Flywheels   
     *  for intaking    elevator          |
     *             \  /                  |
     *              |                    |
     *            /   \                 |
     *  Intake feed   Shooter receive   |
     *  to shooter        Note         |
     *         \      /               /
     *      Aim shooter             /
     *                 \          /
     *                    FINISH
     */

    /** The sequence of commands to move a note from the intake to the shooter and prepare the shooter to shooter.
      *
      * @param intakeRollerSubsystem The intake rollers feed the note to the shooter
      * @param elevatorSubsystem The elevator is lowered
      * @param shooterFeederSubsystem The shooter receives the note TODO: Change for r2
      * @param shooterFlywheelSubsystem The shooter flywheel gets to speed so it can be ready to fire
      * @param shooterPivotSubsystem The pivot lowers for intaking, and then re-angles for firing
      * @param ledSubsystem The leds signal that the note is being fed
    */
    public ShootModeSequence(
        IntakeRollersSubsystem intakeRollerSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ShooterFeederSubsystem shooterFeederSubsystem,
        ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
        ShooterPivotSubsystem shooterPivotSubsystem,
        LEDSubsystem ledSubsystem
    ) {
        addCommands(
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem).alongWith(
                new SequentialCommandGroup(
                    new ShooterPivotSetAngleCommand(shooterPivotSubsystem, FEED_ANGLE).alongWith(
                        new ElevatorToGroundCommand(elevatorSubsystem)
                    ),
                    new ParallelDeadlineGroup(new ShooterFeedLoadCommand(shooterFeederSubsystem),
                                                new IntakeRollerFeedCommand(intakeRollerSubsystem),
                                                new InstantCommand(() -> ledSubsystem.setNoteMode(
                                                NotePosition.TRANSFER_TO_SHOOTER
                                                ))),
                    new InstantCommand(() -> ledSubsystem.setNoteMode(NotePosition.SHOOTER_HOLDING)),
                    new ShooterPivotSetAngleCommand(
                        shooterPivotSubsystem, Units.degreesToRadians(20) //STUB FOR AUTOAIM
                    ), 
                    new InstantCommand(() -> ledSubsystem.setNoteMode(NotePosition.SHOOTER_READY_TO_SHOOT))
                )
            )
        );
    }
}
