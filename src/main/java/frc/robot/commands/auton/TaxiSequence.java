package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** TaxiSequence. */
public class TaxiSequence extends BaseAutonSequence {
    private final ChoreoTrajectory trajectory = Choreo.getTrajectory("REAL2M");

    /** Robot taxis 2 meters backwards. */
    public TaxiSequence(IntakePivotSubsystem intakePivotSubsystem, 
                        IntakeRollersSubsystem intakeRollersSubsystem, 
                        ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
                        ShooterPivotSubsystem shooterPivotSubsystem, 
                        ElevatorSubsystem elevatorSubsystem, 
                        SwerveSubsystem swerveSubsystem, 
                        LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, 
              shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);
        
        // reset robot start pose
        ((SwerveSubsystem) swerveSubsystem).resetPose(trajectory.getInitialPose());

        addCommands(
            followPath(trajectory),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }
}