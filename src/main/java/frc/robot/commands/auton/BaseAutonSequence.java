package frc.robot.commands.auton;
 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.pivot.IntakePivotExtendedCommand;
import frc.robot.commands.intake.pivot.IntakePivotVerticalCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.sequences.ShootModeSequence;
import frc.robot.commands.shooter.feed.ShooterFeedShootCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;


public class BaseAutonSequence extends SequentialCommandGroup{

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollersSubsystem;
    private final ShooterFeederSubsystem shooterFeederSubsystem; 
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController thetaController;
    private PIDController xPID;
    private PIDController yPID;
    private boolean isRed;
    private int driveforwardtime;

    public BaseAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, 
                             ShooterFeederSubsystem shooterFeederSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, 
                             ElevatorSubsystem elevatorSubsystem, 
                             SwerveSubsystem swerveSubsystem,
                             LEDSubsystem ledSubsystem){
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.shooterFeederSubsystem = shooterFeederSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem,intakePivotSubsystem);

        isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    
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
            () -> isRed,
            swerveSubsystem
            );
        return swerveCommand;
    }

    public SequentialCommandGroup goIntake(ChoreoTrajectory intaketraj){
        return followPath(intaketraj)
        .andThen(new IntakePivotExtendedCommand(intakePivotSubsystem))
        .andThen(new IntakeRollerIntakeCommand(intakeRollersSubsystem, ledSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveforwardtime)))
        .andThen(new IntakeRollerIntakeCommand(intakeRollersSubsystem, ledSubsystem))//just in case the note isn't fully intaken above
        .andThen(new IntakeRollerFeedCommand(intakeRollersSubsystem))
        .andThen(new IntakePivotVerticalCommand(intakePivotSubsystem));
    }

    public SequentialCommandGroup goShoot(ChoreoTrajectory shoottraj){
        return followPath(shoottraj)
        .andThen(new ShootModeSequence(intakeRollersSubsystem, elevatorSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, ledSubsystem))
        .andThen(new ShooterFeedShootCommand(shooterFeederSubsystem))
        .andThen(new ShooterFlywheelStopCommand(shooterFlywheelSubsystem));
    }

}