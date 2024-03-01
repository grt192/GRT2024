package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
=======
import edu.wpi.first.math.geometry.Rotation2d;
>>>>>>> b461370 (1-4 peice crauton)
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Middle3PieceSequence extends BaseAutonSequence{

    private final ChoreoTrajectory starttopiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
    private final ChoreoTrajectory piece1topiece2 = Choreo.getTrajectory("D3-SpeakerNoteToAmpNote");
   
    public Middle3PieceSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, ledSubsystem);
        ((SwerveSubsystem) swerveSubsystem).resetPose(starttopiece1.getInitialPose());

        addCommands(
            shoot(),
            goIntakeNoOvershoot(starttopiece1, true),
            shoot(),
            goIntakeNoOvershoot(piece1topiece2, true),
            new SetCalculatedAngleCommand(swerveSubsystem),
            shoot()
        );
    }
}