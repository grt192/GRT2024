package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** BottomPreloadedSequence.*/
public class BottomPreloadedSequence extends BaseAutonSequence {

    private Pose2d initPose = SwerveConstants.IS_RED 
                                ? new Pose2d(new Translation2d(15.11, 4.11), new Rotation2d(Math.PI))  
                                : new Pose2d(new Translation2d(1.43, 4.11), new Rotation2d(0));

    /** Starts: bottom (furthest from amp). Shoots preloaded note only.*/                                
    public BottomPreloadedSequence(IntakePivotSubsystem intakePivotSubsystem, 
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
            new SetCalculatedAngleCommand(swerveSubsystem),
            shoot()
        );
    }
}