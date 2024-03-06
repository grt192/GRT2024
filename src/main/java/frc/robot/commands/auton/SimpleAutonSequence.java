package frc.robot.commands.auton;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SimpleAutonSequence extends BaseAutonSequence{

    private ChoreoTrajectory preloadedtraj;
    private ChoreoTrajectory intaketraj;
    private ChoreoTrajectory speakertraj;
    private Rotation2d shootangle1;
    private Rotation2d shootangle2;
   
    public SimpleAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                               ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, ledSubsystem);

        addCommands(
            goShoot(preloadedtraj),
            goIntake(intaketraj),
            goShoot(speakertraj)
        );
    }
}