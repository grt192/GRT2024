package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

@FunctionalInterface
public interface AutonFactoryFunction {
    Command create(
        IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
        ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
        ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, 
        LEDSubsystem ledSubsystem
    );
}
