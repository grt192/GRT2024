package frc.robot.commands.auton;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.pivot.IntakePivotExtendedCommand;
import frc.robot.commands.intake.pivot.IntakePivotVerticalCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOutakeCommand;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;


public class BaseAutonSequence extends SequentialCommandGroup{

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollersSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private PIDController xPID;
    private PIDController yPID;
    private BooleanSupplier isRed;
    private final PIDController thetaController;

    public BaseAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, SwerveSubsystem swerveSubsystem){
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem,intakePivotSubsystem);

    
        swerveSubsystem = new SwerveSubsystem();

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(12, 0, 0);
        thetaController = new PIDController(3, 0, 0);
    }

    public Command followPath(ChoreoTrajectory traj){
        Command swerveCommand = Choreo.choreoSwerveCommand(
            traj,
            swerveSubsystem::getRobotPosition,
            xPID, 
            yPID, 
            thetaController, 
            ((ChassisSpeeds speeds) -> {swerveSubsystem.setChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, 
                speeds.omegaRadiansPerSecond
                );
            System.out.println(speeds.vxMetersPerSecond);}),
            isRed,
            swerveSubsystem
            );
        return swerveCommand;
    }

    public SequentialCommandGroup goIntake(ChoreoTrajectory traj){
        return followPath(traj)
        .andThen(new IntakePivotExtendedCommand(intakePivotSubsystem))
        .andThen(new IntakeRollerIntakeCommand(intakeRollersSubsystem))
        .andThen(new IntakePivotVerticalCommand(intakePivotSubsystem));
    }
    
    public SequentialCommandGroup goToSpeaker(ChoreoTrajectory traj){
        return followPath(traj)
        .andThen(new IntakePivotExtendedCommand(intakePivotSubsystem))
        .andThen(new IntakeRollerOutakeCommand(intakeRollersSubsystem))
        .andThen(new IntakePivotVerticalCommand(intakePivotSubsystem));
    }

}