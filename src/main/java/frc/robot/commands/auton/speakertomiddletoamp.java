package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class speakertomiddletoamp extends BaseAutonSequence{

    private ChoreoTrajectory middlenoteintaketraj;
    private ChoreoTrajectory ampnoteintaketraj;
    private ChoreoTrajectory shootampnotetraj;
   
    public speakertomiddletoamp(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, ShooterFeederSubsystem shooterFeederSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem) {
        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem);

        
        middlenoteintaketraj = Choreo.getTrajectory("speakerstarttomiddlenote");
        ampnoteintaketraj = Choreo.getTrajectory("speakermiddlenotetoampnote");
        shootampnotetraj = Choreo.getTrajectory("shootampnotetraj");

        addCommands(
            shoot(), 
            goIntake(middlenoteintaketraj),
            shoot(),
            goIntake(ampnoteintaketraj),
            goShoot(shootampnotetraj)
        );
    }
}