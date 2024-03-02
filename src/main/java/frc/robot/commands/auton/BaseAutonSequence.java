package frc.robot.commands.auton;

import static frc.robot.Constants.SwerveConstants.IS_RED;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorToIntakeCommand;
import frc.robot.commands.intake.pivot.IntakePivotExtendedCommand;
import frc.robot.commands.intake.pivot.IntakePivotMiddleCommand;
import frc.robot.commands.intake.pivot.IntakePivotVerticalCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;

/** 
 * The base autonomous sequence that other autons extend. This class provides functions that abstract shared tasks 
 * between autons
 */
public class BaseAutonSequence extends SequentialCommandGroup {

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollerSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController thetaController;
    private PIDController xPID;
    private PIDController yPID;
    private boolean isRed;
    private double driveforwardtime = 1;
    private double shortdriveforwardtime = .5;
    private double intaketime = 3;
    

    public BaseAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ElevatorSubsystem elevatorSubsystem, BaseSwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem){
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollerSubsystem = intakeRollersSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.swerveSubsystem = (SwerveSubsystem) swerveSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem, intakePivotSubsystem);

        // isRed = false ; //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);
        thetaController = new PIDController(3.5, 0, 0);

        addCommands(
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem).withTimeout(3)
        );
    }

    public Command followPath(ChoreoTrajectory traj){
        //swerveSubsystem.resetPose(traj.getInitialPose());
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
            }),
            () -> IS_RED,
            swerveSubsystem
            );
        return swerveCommand;
    }

    public Command goIntake(ChoreoTrajectory intaketraj, Boolean extendwhilemoving){
        
        if (extendwhilemoving){
            return followPath(intaketraj).alongWith(
                new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                    new IntakePivotMiddleCommand(intakePivotSubsystem, 1)
                )
            ).andThen(
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveforwardtime)),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.15),
                new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
                );
        }
        else {
            return followPath(intaketraj).alongWith(
                new ElevatorToIntakeCommand(elevatorSubsystem)
            ).andThen(
                new IntakePivotMiddleCommand(intakePivotSubsystem, 1),
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveforwardtime)),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.3),
                new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
                );
        }
    }

    public Command goIntakeNoOvershoot(ChoreoTrajectory intaketraj, Boolean extendwhilemoving){
        if (extendwhilemoving){
            return followPath(intaketraj).alongWith(
                new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                    new IntakePivotMiddleCommand(intakePivotSubsystem, 1)
                )
            ).andThen(
                new ParallelDeadlineGroup(
                    new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem).withTimeout(3),
                    new DriveForwardCommand(swerveSubsystem)),
                new IntakeRollerFeedCommand(intakeRollerSubsystem, .4).until(intakeRollerSubsystem::backSensorNow),
                new IntakeRollerFeedCommand(intakeRollerSubsystem, .5).withTimeout(.15)
                // new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
                );
        }
        else {
            return followPath(intaketraj).alongWith(
                new ElevatorToIntakeCommand(elevatorSubsystem)
            ).andThen(
                new IntakePivotMiddleCommand(intakePivotSubsystem, 1),
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(shortdriveforwardtime)),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow),
                new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.3),
                new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
                );
        }
    }

     public SequentialCommandGroup shoot(){
        return new ShooterPivotAimCommand(shooterPivotSubsystem).andThen(
            // new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem).withTimeout(2), //wait to hit max speed?
            new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.5)
            // new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        ) ;
    }

    public SequentialCommandGroup goShoot(ChoreoTrajectory shoottraj){
        return followPath(shoottraj)
        .andThen(new SetCalculatedAngleCommand(swerveSubsystem))
        .andThen(shoot());
    }

}