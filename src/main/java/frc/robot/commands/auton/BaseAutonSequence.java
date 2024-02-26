package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorToChuteCommand;
import frc.robot.commands.intake.pivot.IntakePivotExtendedCommand;
import frc.robot.commands.intake.pivot.IntakePivotVerticalCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOutakeCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** 
 * The base autonomous sequence that other autons extend. This class provides functions that abstract shared tasks 
 * between autons
 */
public class BaseAutonSequence extends SequentialCommandGroup {

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollersSubsystem;
    private final ShooterFeederSubsystem shooterFeederSubsystem;
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
    private double intaketime = 3;

    public BaseAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, ShooterFeederSubsystem shooterFeederSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ElevatorSubsystem elevatorSubsystem, BaseSwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem){
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.swerveSubsystem = (SwerveSubsystem) swerveSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem, intakePivotSubsystem);

        isRed = false ; //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);
        thetaController = new PIDController(3.5, 0, 0);
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
            () -> isRed,
            swerveSubsystem
            );
        return swerveCommand;
    }

    // public Command setAngle(double xpower, double ypower, Rotation2d angle){
    //     Command setDrivePowerswithHeadingLock = swerveSubsystem.setDrivePowerswithHeadingLock(xpower, ypower, angle);
    //     return setDrivePowerswithHeadingLock;
    // }

    public SequentialCommandGroup goIntake(ChoreoTrajectory intaketraj){
        return followPath(intaketraj)
                //.andThen(new ElevatorToChuteCommand(elevatorSubsystem))
                .andThen(new IntakePivotExtendedCommand(intakePivotSubsystem))
                .andThen(new IntakeRollerIntakeCommand(intakeRollersSubsystem, ledSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveforwardtime)));
                //.andThen(new IntakeRollerFeedCommand(intakeRollersSubsystem));
                //.andThen(new IntakePivotVerticalCommand(intakePivotSubsystem));
    }

    public SequentialCommandGroup goIntakeNoOvershoot(ChoreoTrajectory intaketraj){
        return followPath(intaketraj)
                //.andThen(new ElevatorToChuteCommand(elevatorSubsystem))
                .andThen(new IntakePivotExtendedCommand(intakePivotSubsystem))
                .andThen(new IntakeRollerIntakeCommand(intakeRollersSubsystem, ledSubsystem).withTimeout(intaketime));
                //.andThen(new IntakeRollerFeedCommand(intakeRollersSubsystem));
                //.andThen(new IntakePivotVerticalCommand(intakePivotSubsystem));
    }

     public SequentialCommandGroup shoot(){
        return new SequentialCommandGroup(null);
        // return new ShootModeSequence(intakeRollersSubsystem, elevatorSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem)
        // .andThen(new ShooterFeedShootCommand(shooterFeederSubsystem))
        // .andThen(new ShooterFlywheelStopCommand(shooterFlywheelSubsystem));
    }

    public SequentialCommandGroup goShoot(ChoreoTrajectory shoottraj){
        return followPath(shoottraj)
        .andThen(shoot());
    }

}