package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Interface used by the AutonChooser to construct and auton sequence. */
@FunctionalInterface
public interface AutonFactoryFunction {
    /** Constructs chosen auton sequence. */
    Command create(
        IntakePivotSubsystem intakePivotSubsystem, IntakeRollerSubsystem intakeRollersSubsystem, 
        ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
        ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, 
        LightBarSubsystem lightBarSubsystem, FieldManagementSubsystem fmsSubsystem
    );
}