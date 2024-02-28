package frc.robot.commands.auton;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.sequences.ShootModeSequence;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BottomPreloadedSequence extends BaseAutonSequence{

    private Rotation2d preloadedShootAngle = new Rotation2d(2.19);
   
    public BottomPreloadedSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                               ShooterFeederSubsystem shooterFeederSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, ledSubsystem);

        addCommands(
            new SetHardAngleCommand(swerveSubsystem, preloadedShootAngle),
            new ShootModeSequence(intakeRollersSubsystem, elevatorSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, ledSubsystem)
        );
    }
}