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

/** Middle2PieceSequence. */
public class Middle2PieceSequence extends BaseAutonSequence {

    private final ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
    private Pose2d initPose = SwerveConstants.IS_RED 
                                ? new Pose2d(new Translation2d(15.151, 5.55), new Rotation2d(Math.PI)) 
                                : new Pose2d(new Translation2d(1.389, 5.55), new Rotation2d(0));

    /** Starts: right in front of subwoofer. Shoots preloaded, intakes middle note, shoots note. */
    public Middle2PieceSequence(IntakePivotSubsystem intakePivotSubsystem, 
                                                  IntakeRollersSubsystem intakeRollersSubsystem, 
                                                  ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
                                                  ShooterPivotSubsystem shooterPivotSubsystem, 
                                                  ElevatorSubsystem elevatorSubsystem, 
                                                  SwerveSubsystem swerveSubsystem, 
                                                  LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, 
                 shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);
        
        // reset robot start pose to resulting pose after preloaded shot
        ((SwerveSubsystem) swerveSubsystem).resetPose(initPose);

        addCommands(
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}