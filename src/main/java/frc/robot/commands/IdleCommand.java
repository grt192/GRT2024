package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.NotePosition;

/** DEPRECATED Sets the Robot to its "idle" position. */
public class IdleCommand extends ParallelCommandGroup {

    /** Sets the robot to its "idle" position.
     *
     * @param intakePivotSubsystem Stows the intake pivot. TODO: implement
     * @param intakeRollersSubsystem Stops the intake rollers.
     * @param elevatorSubsystem Brings the elevator to ground level.
     * @param shooterPivotSubsystem Brings the shooter to horizontal. TODO: implement
     * @param shooterFlywheelSubsystem Stops the shooter flywheels.
     * @param climbSubsystem Lowers climb. TODO: implement
     * @param ledSubsystem Resets the LEDs depending on where the note is.
     */
    @Deprecated
    public IdleCommand(IntakePivotSubsystem intakePivotSubsystem,
                       IntakeRollerSubsystem intakeRollersSubsystem,
                       ElevatorSubsystem elevatorSubsystem,
                       ShooterPivotSubsystem shooterPivotSubsystem,
                       ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                       ClimbSubsystem climbSubsystem,
                       LEDSubsystem ledSubsystem) {
        // addRequirements(intakePivotSubsystem, intakeRollersSubsystem,
        //                 elevatorSubsystem,
        //                 shooterPivotSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem,
        //                 climbSubsystem
        // );

        addCommands(new InstantCommand(() -> intakeRollersSubsystem.setRollSpeeds(0, 0), intakeRollersSubsystem),
                    // new ElevatorToGroundCommand(elevatorSubsystem),
                    new ShooterFlywheelStopCommand(shooterFlywheelSubsystem),
                    new InstantCommand(() -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem),
                    new InstantCommand(() -> {
                        if (intakeRollersSubsystem.getFrontSensorReached()) {
                            ledSubsystem.setNoteMode(NotePosition.INTAKE_HOLDING);
                        } else {
                            ledSubsystem.setNoteMode(NotePosition.NONE);
                        }
                    })//,
                    // new ClimbLowerCommand(climbSubsystem) // NOT USING CLIMB FOR NOW
        );
    }
}
