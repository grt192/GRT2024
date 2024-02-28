package frc.robot.commands.auton;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TopPreloadedSequence extends BaseAutonSequence{

    private double targetRads = SwerveConstants.IS_RED ? Math.PI - 2.18 : -2.18;
    private Rotation2d preloadedShootAngle = new Rotation2d(targetRads);
   
    public TopPreloadedSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, ledSubsystem);

        addCommands(
            new SetHardAngleCommand(swerveSubsystem, preloadedShootAngle),
            shoot()
        );
    }
}