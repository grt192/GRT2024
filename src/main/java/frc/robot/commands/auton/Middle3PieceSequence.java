package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Middle3PieceSequence extends BaseAutonSequence{

    private final ChoreoTrajectory starttopiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
    private final ChoreoTrajectory piece1topiece2 = Choreo.getTrajectory("D3-SpeakerNoteToAmpNote");
   
    public Middle3PieceSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LightBarSubsystem lightBarSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);
        ((SwerveSubsystem) swerveSubsystem).resetPose(starttopiece1.getInitialPose());

        addCommands(
            shoot(),
            goIntake(starttopiece1),
            shoot(),
            goIntake(piece1topiece2),
            shoot(),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}