package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Top2PieceSequence extends BaseAutonSequence{

    // private double targetRads = SwerveConstants.IS_RED ? Math.PI - 2.18 : -2.18;
    // private Rotation2d preloadedShootAngle = new Rotation2d(targetRads);

    private Pose2d initPose = SwerveConstants.IS_RED ? new Pose2d(new Translation2d(15.11, 7.01), new Rotation2d(Math.PI)) : new Pose2d(new Translation2d(1.43, 7.01), new Rotation2d(0));
    private final ChoreoTrajectory starttopiece1 = Choreo.getTrajectory("C1-AmpStartToAmpNote");
   
    public Top2PieceSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                               ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, ledSubsystem);
        ((SwerveSubsystem) swerveSubsystem).resetPose(initPose);

        addCommands(
            new SetCalculatedAngleCommand(swerveSubsystem),
            shoot(),
            goIntakeNoOvershoot(starttopiece1, true),
            new SetCalculatedAngleCommand(swerveSubsystem),
            shoot(),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}