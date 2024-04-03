package frc.robot.commands.auton;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.elevator.ElevatorToEncoderZeroCommand;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotAimCommand;
import frc.robot.commands.swerve.AutoIntakeSequence;
import frc.robot.commands.swerve.AutonNoteAlignCommand;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.GRTUtil;
import frc.robot.vision.NoteDetectionWrapper;


/** 
 * The base autonomous sequence that other autons extend. This class provides functions that abstract shared tasks
 * between autons.
 */
public class AutonBuilder {

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClimbSubsystem climbSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetectionWrapper noteDetector;
    private final LightBarSubsystem lightBarSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private final PIDController thetaController;
    private final PIDController xPID;
    private final PIDController yPID;

    //TODO: have auton follow alternate path if no note intaken.
    /** Constructs a {@link AutonBuilder} with auton-abstracted functions. */
    public AutonBuilder(IntakePivotSubsystem intakePivotSubsystem,
                             IntakeRollerSubsystem intakeRollersSubsystem,
                             ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                             ShooterPivotSubsystem shooterPivotSubsystem,
                             ElevatorSubsystem elevatorSubsystem,
                             ClimbSubsystem climbSubsystem,
                             SwerveSubsystem swerveSubsystem,
                             NoteDetectionWrapper noteDetector,
                             LightBarSubsystem lightBarSubsystem,
                             FieldManagementSubsystem fmsSubsystem) {
        this.intakePivotSubsystem = intakePivotSubsystem; 
        this.intakeRollerSubsystem = intakeRollersSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.climbSubsystem = climbSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.noteDetector = noteDetector;
        this.lightBarSubsystem = lightBarSubsystem;
        this.fmsSubsystem = fmsSubsystem;

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);
        thetaController = new PIDController(3.5, 0, 0);
    }

    /**
     * Follows trajectory.
     *
     * @param trajectory ChoreoTrajectory
     * @return followPath command
     */
    public Command followPath(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
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
    }

    /**
     * Follows trajectory to intake.
     *
     * @param intakeTrajectory ChoreoTrajectory
     * @return goIntake Command
     */
    public Command goIntake(ChoreoTrajectory intakeTrajectory) {
        return followPath(intakeTrajectory).andThen(
            new ParallelRaceGroup(
                new AutonNoteAlignCommand(swerveSubsystem, intakeRollerSubsystem, noteDetector),
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
            ).withTimeout(1),
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem).until(
                ()  -> intakeRollerSubsystem.getRockwellSensorValue()).withTimeout(2)
            // new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
            //     .alongWith(new DriveForwardCommand(swerveSubsystem).until(
            //         intakeRollerSubsystem::getFrontSensorValue).until(
            //             intakeRollerSubsystem::getBackSensorReached).withTimeout(1))
        );
    }

    /** 
     * Shoots at calculated robot angle and shooter angle.
     *
     * @return shoot command
     */
    public Command shoot() {
        return new SetCalculatedAngleCommand(swerveSubsystem).withTimeout(1)
            .andThen(new IntakeRollerFeedCommand(intakeRollerSubsystem).until(
                () -> !intakeRollerSubsystem.getRockwellSensorValue())
            .andThen(new IntakeRollerFeedCommand(intakeRollerSubsystem)).withTimeout(.5)
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
            new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem).alongWith(
                new SetCalculatedAngleCommand(swerveSubsystem),
                new ShooterPivotAimCommand(shooterPivotSubsystem),
                new IntakePivotSetPositionCommand(intakePivotSubsystem, 1)
            ).withTimeout(2)
        );
        autonSequence.addCommands(
            (new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem, lightBarSubsystem)).alongWith(
                new ShooterPivotAimCommand(shooterPivotSubsystem),
                new SequentialCommandGroup(commands)
            )
            // commands
        );
        autonSequence.addCommands(
            new WaitCommand(2),
            new ShooterFlywheelStopCommand(shooterFlywheelSubsystem)
        );

        return autonSequence;
    }

    /**Starts furthest away from amp in SPECIAL START POSITION, sweeps center notes starting furthest away from amp.*/
    public SequentialCommandGroup getBottomBottomCenterDisruptor() {
        ChoreoTrajectory trajectory = Choreo.getTrajectory("BottomBottomCenterDisruptor");
        return new SequentialCommandGroup(
            resetSwerve(GRTUtil.mirrorAcrossField(trajectory.getInitialPose(), fmsSubsystem::isRedAlliance)),
            followPath(trajectory));
    }

    /** starts furthest away from amp in SPECIAL START POSITION, sweeps center notes starting closest to amp. */
    public SequentialCommandGroup getBottomTopCenterDisruptor() {
        ChoreoTrajectory trajectory = Choreo.getTrajectory("BottomTopCenterDistruptor");
        return new SequentialCommandGroup(
            resetSwerve(GRTUtil.mirrorAcrossField(trajectory.getInitialPose(), fmsSubsystem::isRedAlliance)),
            new InstantCommand(() -> climbSubsystem.setSpeeds(-0.2, -0.2)),
            followPath(trajectory));
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
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA1");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /** Starts amp side, shoots preloaded and 2 other wing notes. */
    public SequentialCommandGroup getTopThreePiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA1");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("12");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot()
        );
    }

    /** Starts amp side, shoots preloaded and 3 other wing notes. */
    public SequentialCommandGroup getTopFourPiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA1");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("12");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("23");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            shoot()
        );

    }

    /** Starts by the amp, shoots preloaded, gets 2 closest to amp wing notes, 
     * then closest 2 to amp center note. */
    public SequentialCommandGroup getTopTwoPieceThenCenter2() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA1");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("12");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("24");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("4X");
        ChoreoTrajectory wingToPiece4 = Choreo.getTrajectory("X5");
        ChoreoTrajectory piece4ToWing = Choreo.getTrajectory("5X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            goShoot(piece3ToWing),
            goIntake(wingToPiece4),
            goShoot(piece4ToWing)
        );

    }

    /** Starts by the amp, shoots preloaded, gets 2 closest to amp wing notes, 
     * then closest to amp center note. */
    public SequentialCommandGroup getTopTwoPieceThenCenter1() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA1");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("12");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("24");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("4X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            goShoot(piece3ToWing)
        );

    }

    /** Starts by the amp, shoots preloaded, then gets two center notes closest to amp. */
    public SequentialCommandGroup getTopCenterTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("OA4");
        ChoreoTrajectory piece1ToWing = Choreo.getTrajectory("4X");
        ChoreoTrajectory wingToPiece2 = Choreo.getTrajectory("X5");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("5X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            goShoot(piece1ToWing),
            goIntake(wingToPiece2),
            goShoot(piece3ToWing)
        );

    }
    
    /** Starts in front of the subwoofer, shoots preloaded, then gets two center notes closest to amp. */
    public SequentialCommandGroup getMiddleCenterTwoPiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M4");
        ChoreoTrajectory piece1ToWing = Choreo.getTrajectory("4X");
        ChoreoTrajectory wingToPiece2 = Choreo.getTrajectory("X5");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("5X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            goShoot(piece1ToWing),
            goIntake(wingToPiece2),
            goShoot(piece3ToWing)
        );
        
    }

    /** Starts at subwoofer. Shoots preloaded note, intakes middle  note, shoots note. */
    public SequentialCommandGroup getMiddleTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M2");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /** Starts source side. Shoots preloaded note, intakes bottom note, shoots note. */
    public SequentialCommandGroup getBottomTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("S3");

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

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M2");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("21");

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

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M2");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("21");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("13");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            shoot()
        );
    }

    /** Starts in front of subwoofer, shoots preloaded, gets 2 closest to amp wing notes, 
     * then closest to amp center note. */
    public SequentialCommandGroup getMiddleTwoPieceThen1TopCenter() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M2");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("21");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("14");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("4X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            goShoot(piece3ToWing)
        );
    }

    /** Starts in front of subwoofer, shoots preloaded, gets 2 closest to amp wing notes, 
     * then 2 closest to amp center note. */
    public SequentialCommandGroup getMiddleTwoPieceThen2TopCenter() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M2");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("21");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("14");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("4X");
        ChoreoTrajectory wingToPiece4 = Choreo.getTrajectory("X5");
        ChoreoTrajectory piece4ToWing = Choreo.getTrajectory("5X");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntake(piece2ToPiece3),
            goShoot(piece3ToWing),
            goIntake(wingToPiece4),
            goShoot(piece4ToWing)
        );
    }

    /** Drives 2 meters away from the alliance wall. */
    public Command getTaxiTwoMeters() {
        return AutoBuilder.pathfindToPose(
            swerveSubsystem.getRobotPosition().plus(
                new Transform2d(new Translation2d(fmsSubsystem.isRedAlliance() ? -2 : 2, 0), new Rotation2d())
            ), 
            new PathConstraints(
            2.0, 2.0, 
                    Units.degreesToRadians(720), Units.degreesToRadians(1080)
                    ), 
            0, 
            0.0
                );
    }
}