package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Descriptive javadoc here. */ 
public class Top2PieceSequence extends BaseAutonSequence {

    // private double targetRads = SwerveConstants.IS_RED ? Math.PI - 2.18 : -2.18;
    // private Rotation2d preloadedShootAngle = new Rotation2d(targetRads);

    private Pose2d initPose = SwerveConstants.IS_RED
                                ? new Pose2d(new Translation2d(15.11, 7.01), new Rotation2d(Math.PI))
                                : new Pose2d(new Translation2d(1.43, 7.01), new Rotation2d(0));

    private final ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("C1-AmpStartToAmpNote");
   
    /** Constructs a {@link Top2PieceSequence}. */
    public Top2PieceSequence(IntakePivotSubsystem intakePivotSubsystem,
                             IntakeRollersSubsystem intakeRollersSubsystem, 
                             ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                             ShooterPivotSubsystem shooterPivotSubsystem, 
                             ElevatorSubsystem elevatorSubsystem,
                             SwerveSubsystem swerveSubsystem,
                             LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem,
              shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);

        ((SwerveSubsystem) swerveSubsystem).resetPose(initPose);

        addCommands(
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}