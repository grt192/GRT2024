package frc.robot.commands.auton;

import static frc.robot.Constants.SwerveConstants.IS_RED;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorToIntakeCommand;
import frc.robot.commands.intake.pivot.IntakePivotMiddleCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;


/** The base autonomous sequence that other autons extend. This class provides functions that abstract shared tasks 
 * between autons.
 */
public class BaseAutonSequence extends SequentialCommandGroup {

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollerSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final LightBarSubsystem lightBarSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController thetaController;
    private PIDController xPID;
    private PIDController yPID;
    private double driveforwardtime = 1;
    private double shortdriveforwardtime = .5;
    private double intaketime = 3;
    

    public BaseAutonSequence(IntakePivotSubsystem intakePivotSubsystem, IntakeRollersSubsystem intakeRollersSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem, ShooterPivotSubsystem shooterPivotSubsystem, ElevatorSubsystem elevatorSubsystem, BaseSwerveSubsystem swerveSubsystem, LightBarSubsystem lightBarSubsystem){
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollerSubsystem = intakeRollersSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        this.swerveSubsystem = (SwerveSubsystem) swerveSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem, intakePivotSubsystem);

        // isRed = false ; //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);
        thetaController = new PIDController(3.5, 0, 0);

        addCommands(
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem).withTimeout(3)
        );
    }

    /**
     * Follows trajectory.
     *
     * @param trajectory ChoreoTrajectory
     * 
     * @return followPath command
     */
    public Command followPath(ChoreoTrajectory trajectory) {
        //swerveSubsystem.resetPose(trajectory.getInitialPose());
        Command swerveCommand = Choreo.choreoSwerveCommand(
            trajectory,
            swerveSubsystem::getRobotPosition,
            xPID, 
            yPID, 
            thetaController, 
            ((ChassisSpeeds speeds) -> {
                swerveSubsystem.setChassisSpeeds(
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

    public Command goIntake(ChoreoTrajectory intakeTraj){
        return followPath(intakeTraj).alongWith(
            new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                new IntakePivotMiddleCommand(intakePivotSubsystem, 1)
            )
        ).andThen(
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem).raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveforwardtime)),
            new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow),
            new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.15),
            new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
        );
    }

    public SequentialCommandGroup shoot() {
        return new ShooterPivotAimCommand(shooterPivotSubsystem).andThen(
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem).withTimeout(2), //wait to hit max speed?
            new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.3),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );
    }

    public SequentialCommandGroup goShoot(ChoreoTrajectory shootTrajectory) {
        return followPath(shootTrajectory)
        .andThen(shoot());
    }
}