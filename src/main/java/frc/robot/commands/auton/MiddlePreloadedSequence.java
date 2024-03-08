package frc.robot.commands.auton;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Middle2PieceSequence. */
public class MiddlePreloadedSequence extends BaseAutonSequence {

    /** Starts: right in front of subwoofer. Shoots preloaded, intakes middle note, shoots note. */
    public MiddlePreloadedSequence(IntakePivotSubsystem intakePivotSubsystem, 
                                   IntakeRollersSubsystem intakeRollersSubsystem, 
                                   ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
                                   ShooterPivotSubsystem shooterPivotSubsystem, 
                                   ElevatorSubsystem elevatorSubsystem, 
                                   SwerveSubsystem swerveSubsystem, 
                                   LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, 
              shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);

        addCommands(
            shoot()
        );
    }
}