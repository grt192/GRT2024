package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Middle2PieceDisruptorSequence. */
public class BottomPlayoffsSequence extends BaseAutonSequence {
    private final ChoreoTrajectory trajectory = Choreo.getTrajectory("BottomPLAYOFFS");
   
    /** Starts: Bottom of alliance. Goes to center of the field and pushes around bottom 2 notes. */
    public BottomPlayoffsSequence(IntakePivotSubsystem intakePivotSubsystem,
                             IntakeRollerSubsystem intakeRollersSubsystem,
                             ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                             ShooterPivotSubsystem shooterPivotSubsystem,
                             ElevatorSubsystem elevatorSubsystem,
                             SwerveSubsystem swerveSubsystem,
                             LightBarSubsystem lightBarSubsystem,
                             FieldManagementSubsystem fmsSubsystem) {
                                
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, 
              elevatorSubsystem, swerveSubsystem, lightBarSubsystem, fmsSubsystem);

        // reset robot start pose to resulting pose after preloaded shot
        swerveSubsystem.resetPose(trajectory.getInitialPose());


        addCommands(
            followPath(trajectory)
        );
    }
}