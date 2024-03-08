package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class speakertomiddletoamp extends BaseAutonSequence{

    private ChoreoTrajectory middlenoteintaketraj;
    private ChoreoTrajectory ampnoteintaketraj;
    private ChoreoTrajectory shootampnotetraj;
   
    public speakertomiddletoamp(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                                ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                                ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, LightBarSubsystem lightBarSubsystem) {

        super(intakePivotSubsystem, intakeRollersSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, swerveSubsystem, lightBarSubsystem);
        
        middlenoteintaketraj = Choreo.getTrajectory("Turn90");
        ampnoteintaketraj = Choreo.getTrajectory("speakermiddlenotetoampnote");
        shootampnotetraj = Choreo.getTrajectory("speakerampnoteshoot");

        ((SwerveSubsystem) swerveSubsystem).resetPose(middlenoteintaketraj.getInitialPose());
        System.out.println("X:" + middlenoteintaketraj.getInitialPose().getX());
        System.out.println("Y:" + middlenoteintaketraj.getInitialPose().getY());

        addCommands(

            //shoot(), 
            followPath(middlenoteintaketraj)
            //shoot(),
            //goIntake(ampnoteintaketraj)
            //goShoot(shootampnotetraj)
        );
    }
}