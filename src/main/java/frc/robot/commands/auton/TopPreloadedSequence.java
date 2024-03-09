package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Descriptive javadoc here. */
public class TopPreloadedSequence extends BaseAutonSequence {

    // private double targetRads = SwerveConstants.IS_RED ? Math.PI - 2.18 : -2.18;
    // private Rotation2d preloadedShootAngle = new Rotation2d(targetRads);

    private Pose2d initPose;
   
    /** Constructs a {@link TopPreloadedSequence}. */
    public TopPreloadedSequence(IntakePivotSubsystem intakePivotSubsystem,
                                IntakeRollersSubsystem intakeRollersSubsystem,
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                                ShooterPivotSubsystem shooterPivotSubsystem,
                                ElevatorSubsystem elevatorSubsystem,
                                SwerveSubsystem swerveSubsystem,
                                LightBarSubsystem lightBarSubsystem,
                                FieldManagementSubsystem fmsSubsystem) {
                                
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, 
              elevatorSubsystem, swerveSubsystem, lightBarSubsystem, fmsSubsystem);

        initPose = fmsSubsystem.isRedAlliance()
                 ? new Pose2d(new Translation2d(15.11, 7.01), new Rotation2d(Math.PI))
                 : new Pose2d(new Translation2d(1.43, 7.01), new Rotation2d(0));

        swerveSubsystem.resetPose(initPose);

        addCommands(
            new SetCalculatedAngleCommand(swerveSubsystem),
            shoot()
        );
    }
}