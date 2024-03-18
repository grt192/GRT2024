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
public class AutonBuilder {

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
        return followPath(intakeTrajectory).alongWith(
            //new IntakePivotSetPositionCommand(intakePivotSubsystem, 1)
        ).andThen(
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
                .alongWith(new DriveForwardCommand(swerveSubsystem).until(intakeRollerSubsystem::getFrontSensorValue).until(intakeRollerSubsystem::getBackSensorReached))
        );
    }

    public Command goIntakeNoOvershoot(ChoreoTrajectory intakeTrajectory) {
        return followPath(intakeTrajectory).alongWith(
            //new IntakePivotSetPositionCommand(intakePivotSubsystem, 1)
        ).andThen(
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
                .alongWith(new DriveForwardCommand(swerveSubsystem).withTimeout(.7))
        );
    }

    public Command goAndIntake(ChoreoTrajectory intakeTrajectory) {
        return followPath(intakeTrajectory).alongWith(
            new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
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
            .andThen(new IntakeRollerFeedCommand(intakeRollerSubsystem).until(
                () -> !intakeRollerSubsystem.getRockwellSensorValue()
            )
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
            )
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
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("T1-OffsetAmpStartToAmpNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    public SequentialCommandGroup getTopThreePiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("T1-OffsetAmpStartToAmpNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("T2-AmpNoteToSpeakerNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot()
        );
    }

    public SequentialCommandGroup getTopFourPiece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("T1-OffsetAmpStartToAmpNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("T2-AmpNoteToSpeakerNote");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("T3-SpeakerNoteToBottomNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goAndIntake(piece2ToPiece3),
            shoot()
        );

    }

    public SequentialCommandGroup getTopCenterTwoPiece(){
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("Z1-TopToCenter1");
        ChoreoTrajectory piece1ToWing = Choreo.getTrajectory("Z2-Center1ToWing");
        ChoreoTrajectory wingToPiece2 = Choreo.getTrajectory("Z3-WingToCenter2");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("Z4-Center2ToWing");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            goShoot(piece1ToWing),
            goIntake(wingToPiece2),
            goShoot(piece3ToWing)
        );

    }
    
    public SequentialCommandGroup getMiddleCenterTwoPiece(){
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("Z1-MiddleToCenter1");
        ChoreoTrajectory piece1ToWing = Choreo.getTrajectory("Z2-Center1ToWing");
        ChoreoTrajectory wingToPiece2 = Choreo.getTrajectory("Z3-WingToCenter2");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("Z4-Center2ToWing");

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
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M1-SpeakerStartToSpeakerNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot()
        );
    }

    /** Starts source side. Shoots preloaded note, intakes bottom note, shoots note. */
    public SequentialCommandGroup getBottomTwoPiece() {
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("B1-OffsetBottomNoteStartToBottomNote");

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

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("M2-SpeakerNoteToAmpNote");

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

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("M2-SpeakerNoteToAmpNote");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("M3-AmpToBottomNote");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            shoot(),
            goIntake(piece1ToPiece2),
            shoot(),
            goIntakeNoOvershoot(piece2ToPiece3),
            shoot()
        );
    }

    public SequentialCommandGroup get2TopWingThen1Center1(){
        
        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("M2-SpeakerNoteToAmpNote");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("Z1M3-AmpToCenter1");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("Z2-Center1ToWing");

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

    public SequentialCommandGroup get2TopWingThen2TopCenter() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("M1-SpeakerStartToSpeakerNote");
        ChoreoTrajectory piece1ToPiece2 = Choreo.getTrajectory("M2-SpeakerNoteToAmpNote");
        ChoreoTrajectory piece2ToPiece3 = Choreo.getTrajectory("Z1M3-AmpToCenter1");
        ChoreoTrajectory piece3ToWing = Choreo.getTrajectory("Z2-Center1ToWing");
        ChoreoTrajectory wingToPiece4 = Choreo.getTrajectory("Z3-WingToCenter2");
        ChoreoTrajectory piece4ToWing = Choreo.getTrajectory("Z4-Center2ToWing");

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

    public SequentialCommandGroup getAmpToCenterTop2Piece() {

        ChoreoTrajectory startToPiece1 = Choreo.getTrajectory("Z1-OffsetTopToCenter1");
        ChoreoTrajectory piece1ToWing = Choreo.getTrajectory("Z2-Center1ToWing");
        ChoreoTrajectory wingToPiece2 = Choreo.getTrajectory("Z3-WingToCenter2");
        ChoreoTrajectory piece2ToWing = Choreo.getTrajectory("Z4-Center2ToWing");

        return buildAuton(
            new Pose2d(startToPiece1.getInitialPose().getTranslation(), new Rotation2d()),
            shoot(),
            goIntake(startToPiece1),
            goShoot(piece1ToWing),
            goIntake(wingToPiece2),
            goShoot(piece2ToWing)
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