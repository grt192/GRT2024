package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.elevator.ElevatorToIntakeCommand;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.GRTUtil;


/** 
 * The base autonomous sequence that other autons extend. This class provides functions that abstract shared tasks
 * between autons.
 */
public class AutonBuilder extends SequentialCommandGroup {

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LightBarSubsystem lightBarSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private final PIDController thetaController;
    private final PIDController xPID;
    private final PIDController yPID;

    private double driveForwardTime = 1;

    /** Constructs a {@link AutonBuilder} with auton-abstracted functions. */
    public AutonBuilder(IntakePivotSubsystem intakePivotSubsystem,
                             IntakeRollerSubsystem intakeRollersSubsystem,
                             ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                             ShooterPivotSubsystem shooterPivotSubsystem,
                             ElevatorSubsystem elevatorSubsystem,
                             SwerveSubsystem swerveSubsystem,
                             LightBarSubsystem lightBarSubsystem,
                             FieldManagementSubsystem fmsSubsystem) {
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollerSubsystem = intakeRollersSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        this.fmsSubsystem = fmsSubsystem;

        addRequirements(swerveSubsystem, intakeRollersSubsystem, intakePivotSubsystem);

        // isRed = false ; //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);
        thetaController = new PIDController(3.5, 0, 0);

        addCommands(
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem).withTimeout(
                AutonConstants.SHOOT_FEED_TIME
            )
        );
    }

    /**
     * Follows trajectory.
     *
     * @param trajectory ChoreoTrajectory
     * @return followPath command
     */
    public Command followPath(ChoreoTrajectory trajectory) {
        // swerveSubsystem.resetPose(trajectory.getInitialPose());
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
            fmsSubsystem::isRedAlliance,
            swerveSubsystem
            );
        return swerveCommand;
    }

    /**
     * Follows trajectory to intake.
     *
     * @param intakeTrajectory ChoreoTrajectory
     * @return goIntake Command
     */
    public Command goIntake(ChoreoTrajectory intakeTrajectory) {
        return followPath(intakeTrajectory).alongWith(
            new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                new IntakePivotSetPositionCommand(intakePivotSubsystem, 1)
            )
        ).andThen(
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
                .raceWith(new DriveForwardCommand(swerveSubsystem).withTimeout(driveForwardTime)),
            new IntakePivotSetPositionCommand(intakePivotSubsystem, 0)
        );
    }

    /** 
     * Shoots at calculated robot angle and shooter angle.
     *
     * @return shoot command
     */
    public Command shoot() {
        return new ShooterPivotAimCommand(shooterPivotSubsystem)
            .alongWith(new SetCalculatedAngleCommand(swerveSubsystem))
            .andThen(new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(AutonConstants.SHOOT_FEED_TIME)
        );
    }

    /**
     * Follows trajectory and then shoots at calculated robot angle and shooter angle.
     *
     * @param shootTrajectory ChoreoTrajectory
     * @return goShoot command
     */
    public SequentialCommandGroup goShoot(ChoreoTrajectory shootTrajectory) {
        return followPath(shootTrajectory)
        .andThen(shoot());
    }

    /**
     * Resets the swerve pose estimation to a newPose.
     *
     * @param newPose The pose to reset pose estimation to.
     * @return The command to reset the pose.
     */
    public InstantCommand resetSwerve(Pose2d newPose) {
        return new InstantCommand(() -> swerveSubsystem.resetPose(newPose), swerveSubsystem);
    }

    /**
     * Wraps auton commands with common auton commands.
     *
     * @param initPose The initial pose to reset swerve to.
     * @param commands The commands to run during auton.
     * @return The full auton sequence.
     */
    public SequentialCommandGroup buildAuton(Pose2d initPose, Command... commands) {
        SequentialCommandGroup autonSequence = new SequentialCommandGroup();

        autonSequence.addCommands(
            resetSwerve(GRTUtil.mirrorAcrossField(initPose, fmsSubsystem::isRedAlliance)),
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem)
        );
        autonSequence.addCommands(commands);
        autonSequence.addCommands(
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );

        return autonSequence;
    }

    /** Starts amp side and shoots the preloaded note. */
    public SequentialCommandGroup getTopPreloaded() {
        return buildAuton(AutonConstants.TOP_START_POSE, shoot());
    }

    /** Starts in front of subwoofer and shoots preloaded note. */
    public SequentialCommandGroup getMiddlePreloaded() {
        return buildAuton(AutonConstants.MIDDLE_START_POSE, shoot());
    }

    /** Starts source side and shoots preloaded note. */
    public SequentialCommandGroup getBottomPreloaded() {
        return buildAuton(AutonConstants.BOTTOM_START_POSE, shoot());
    }

    /** Starts amp side. Shoots preloaded note, intakes top note, shoots note. */
    public SequentialCommandGroup getTopTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("C1-AmpStartToAmpNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /** Starts at subwoofer. Shoots preloaded note, intakes middle  note, shoots note. */
    public SequentialCommandGroup getMiddleTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /** Starts source side. Shoots preloaded note, intakes bottom note, shoots note. */
    public SequentialCommandGroup getBottomTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("B1-BottomStartToBottomNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /**
     * Starts: right in front of subwoofer. Shoots preloaded note, intakes middle note, shoots,
     * intakes top note, shoots.
     */
    public SequentialCommandGroup getMiddleThreePiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("D3-SpeakerNoteToAmp");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot()
        );
    }

    /**
     * Starts: right in front of subwoofer. Shoots preloaded note, intakes middle note, shoots,
     * intakes top note, shoots.
     */
    public SequentialCommandGroup getMiddleFourPiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("A1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("D3-SpeakerNoteToAmp");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot()
        );
    }

    /** Starts source side, goes to center of the field and pushes around bottom 2 notes. */
    public SequentialCommandGroup getBottomDisruptor() {

        ChoreoTrajectory trajectory = Choreo.getTrajectory("BottomPLAYOFFS");

        return buildAuton(
            trajectory.getInitialPose(),
            followPath(trajectory)    
        );
    }

    /** Drives forward 2 meters. */
    public Command getTaxi() {

        ChoreoTrajectory trajectory = Choreo.getTrajectory("REAL2M");

        return followPath(trajectory);
    }
}