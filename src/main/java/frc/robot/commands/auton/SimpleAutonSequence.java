package frc.robot.commands.auton;

import com.choreo.lib.ChoreoTrajectory;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SimpleAutonSequence extends BaseAutonSequence{

    private ChoreoTrajectory preloadedtraj;
    private ChoreoTrajectory intaketraj;
    private ChoreoTrajectory speakertraj;

    /**
     * Starts with robot in front of speaker, shoots preloaded note, intake/shoot speaker note, goes to intake/shoot amp note
     * @param intakePivotSubsystem
     * @param intakeRollersSubsystem
     * @param shooterFeederSubsystem
     * @param shooterFlywheelSubsystem
     * @param shooterPivotSubsystem
     * @param elevatorSubsystem
     * @param swerveSubsystem
     * @param ledSubsystem
     */
    public SimpleAutonSequence(IntakePivotSubsystem intakePivotSubsystem, 
                                                IntakeRollersSubsystem intakeRollersSubsystem, 
                                                ShooterFeederSubsystem shooterFeederSubsystem, 
                                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, 
                                                ShooterPivotSubsystem shooterPivotSubsystem, 
                                                ElevatorSubsystem elevatorSubsystem, 
                                                SwerveSubsystem swerveSubsystem, 
                                                LEDSubsystem ledSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, 
                shooterFeederSubsystem, shooterFlywheelSubsystem, 
                shooterPivotSubsystem, elevatorSubsystem, 
                swerveSubsystem, ledSubsystem);

        addCommands(
            goShoot(preloadedtraj), 
            goIntake(intaketraj),
            goShoot(speakertraj)
        );
    }
}