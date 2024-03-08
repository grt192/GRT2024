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

/** Middle4PieceSequence. */
public class Middle4PieceSequence extends BaseAutonSequence {

    private final ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
    private final ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("D3-SpeakerNoteToAmpNote");
    private final ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("D4-AmpToBottomNote");

    private Pose2d initPose = SwerveConstants.IS_RED 
                                            ? new Pose2d(new Translation2d(15.151, 5.55), new Rotation2d(Math.PI)) 
                                            : new Pose2d(new Translation2d(1.389, 5.55), new Rotation2d(0));

    /** Starts: right in front of subwoofer. Shoots preloaded note, intakes middle note, shoots, 
     * intakes top note, shoots, intakes bottom note, shoot. */                                     
    public Middle4PieceSequence(IntakePivotSubsystem intakePivotSubsystem, 
                                                  IntakeRollersSubsystem intakeRollersSubsystem, 
                                                  ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
                                                  ShooterPivotSubsystem shooterPivotSubsystem, 
                                                  ElevatorSubsystem elevatorSubsystem, 
                                                  SwerveSubsystem swerveSubsystem, 
                                                  LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, 
                shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);

        //reset robot start pose to resulting pose after preloaded shot
        ((SwerveSubsystem) swerveSubsystem).resetPose(initPose);

        addCommands(
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            shoot(),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}