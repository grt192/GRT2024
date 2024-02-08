package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.BLUE_SPEAKER_POS;
import static frc.robot.Constants.SwerveConstants.BL_DRIVE;
import static frc.robot.Constants.SwerveConstants.BL_OFFSET;
import static frc.robot.Constants.SwerveConstants.BL_POS;
import static frc.robot.Constants.SwerveConstants.BL_STEER;
import static frc.robot.Constants.SwerveConstants.BR_DRIVE;
import static frc.robot.Constants.SwerveConstants.BR_OFFSET;
import static frc.robot.Constants.SwerveConstants.BR_POS;
import static frc.robot.Constants.SwerveConstants.BR_STEER;
import static frc.robot.Constants.SwerveConstants.FL_DRIVE;
import static frc.robot.Constants.SwerveConstants.FL_OFFSET;
import static frc.robot.Constants.SwerveConstants.FL_POS;
import static frc.robot.Constants.SwerveConstants.FL_STEER;
import static frc.robot.Constants.SwerveConstants.FR_DRIVE;
import static frc.robot.Constants.SwerveConstants.FR_OFFSET;
import static frc.robot.Constants.SwerveConstants.FR_POS;
import static frc.robot.Constants.SwerveConstants.FR_STEER;
import static frc.robot.Constants.SwerveConstants.RED_SPEAKER_POS;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA;
import static frc.robot.Constants.VisionConstants.BACK_LEFT_CAMERA_POSE;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA;
import static frc.robot.Constants.VisionConstants.BACK_RIGHT_CAMERA_POSE;
import static frc.robot.Constants.VisionConstants.FRONT_RIGHT_CAMERA;
import static frc.robot.Constants.VisionConstants.FRONT_RIGHT_CAMERA_POSE;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.vision.ApriltagWrapper;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/** The subsystem that controls the swerve drivetrain. */
public class SwerveSubsystem extends BaseSwerveSubsystem {
    private final AHRS ahrs;

    private final Timer crimer;
    private final Timer ahrsTimer;

    public static final double MAX_VEL = 4.172; //calculated
    public static final double MAX_ACCEL = 3; //STUB
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();
    public static final double MAX_ALPHA = 8; //STUB

    public static final double ANGLE_OFFSET_FOR_AUTO_AIM = Units.degreesToRadians(0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private Rotation2d driverHeadingOffset = new Rotation2d();

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;
    private final ApriltagWrapper[] apriltagWrappers = {
        new ApriltagWrapper(FRONT_RIGHT_CAMERA, FRONT_RIGHT_CAMERA_POSE),
        new ApriltagWrapper(BACK_LEFT_CAMERA, BACK_LEFT_CAMERA_POSE),
        new ApriltagWrapper(BACK_RIGHT_CAMERA, BACK_RIGHT_CAMERA_POSE)
    };

    //heading lock controller
    private final PIDController thetaController;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable = inst.getTable("Testing");
    private final DoublePublisher[] angles = {
        networkTable.getDoubleTopic("module1rot").publish(),
        networkTable.getDoubleTopic("module2rot").publish(),
        networkTable.getDoubleTopic("module3rot").publish(),
        networkTable.getDoubleTopic("module4rot").publish()
    };
    private final DoublePublisher[] velocities = {
        networkTable.getDoubleTopic("module1vel").publish(),
        networkTable.getDoubleTopic("module2vel").publish(),
        networkTable.getDoubleTopic("module3vel").publish(),
        networkTable.getDoubleTopic("module4vel").publish()
    };

    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    public final ShuffleboardTab choreoTab;
    private final Field2d field;

    // private final GenericEntry FLsteer, FLdrive, FRsteer, FRdrive, BLsteer, BLdrive, BRsteer, BRdrive;
    private final GenericEntry robotPos;

    private boolean isRed = false; //DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    /** Constructs a swerve subsystem. */
    public SwerveSubsystem() {
        ahrs = new AHRS(SPI.Port.kMXP);

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET, true);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET, true);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET, true);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET, true);
        
        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

        thetaController = new PIDController(4, 0, 0);
        thetaController.enableContinuousInput(Math.PI, -Math.PI);

        inst.startServer();

        crimer = new Timer();
        crimer.start();

        ahrsTimer = new Timer();
        ahrsTimer.start();

        choreoTab = Shuffleboard.getTab("Auton");
        field = new Field2d();
        choreoTab.add("Field", field)
            .withPosition(0, 0)
            .withSize(6, 4);

        robotPos = choreoTab.add("position", 0.).withPosition(7, 0).getEntry();

        // FLsteer = choreoTab.add("FLsteer", 0.).withPosition(0, 0).getEntry();
        // FLdrive = choreoTab.add("FLdrive", 0.).withPosition(0, 1).getEntry();
        
        // FRsteer = choreoTab.add("FRsteer", 0.).withPosition(1, 0).getEntry();
        // FRdrive = choreoTab.add("FRdrive", 0.).withPosition(1, 1).getEntry();
        
        // BLsteer = choreoTab.add("BLsteer", 0.).withPosition(2, 0).getEntry();
        // BLdrive = choreoTab.add("BLdrive", 0.).withPosition(2, 1).getEntry();
        
        // BRsteer = choreoTab.add("BRsteer", 0.).withPosition(3, 0).getEntry();
        // BRdrive = choreoTab.add("BRdrive", 0.).withPosition(3, 1).getEntry();

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            getGyroHeading(), 
            getModulePositions(),
            new Pose2d(),
            // State measurement standard deviations: [X, Y, theta]
            MatBuilder.fill(Nat.N3(), Nat.N1(), 0.02, 0.02, 0.01),
            // Vision measurement standard deviations: [X, Y, theta]
            MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.01)
        );

        // Configure AutoBuilder
        AutoBuilder.configureHolonomic(
            this::getRobotPosition, 
            this::resetPose, 
            this::getRobotRelativeChassisSpeeds, 
            this::setRobotRelativeDrivePowers, 
            new HolonomicPathFollowerConfig(
                        new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(3.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        FL_POS.getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true)
                ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                // var alliance = DriverStation.getAlliance();
                // if (alliance.isPresent()) {
                //     return alliance.get() == DriverStation.Alliance.Red;
                // }
                return false;
            },
            this
        );
    }

    @Override
    public void periodic() {

        robotPos.setValue(getRobotPosition().getX());
    
        
        // System.out.println(frontLeftModule.getDriveSetpoint());

        // if (crimer.advanceIfElapsed(.1)){
        //     //System.out.println("BR : " + backRightModule.getRawAngle());
        //     System.out.println("BL : " + backLeftModule.getRawAngle());
        // }
    
        // FLsteer.setValue(frontLeftModule.getSteerAmpDraws());
        // FLdrive.setValue(frontLeftModule.getDriveAmpDraws());

        // FRsteer.setValue(frontRightModule.getSteerAmpDraws());
        // FRdrive.setValue(frontRightModule.getDriveAmpDraws());

        // BLsteer.setValue(backLeftModule.getSteerAmpDraws());
        // BLdrive.setValue(backLeftModule.getDriveAmpDraws());

        // BRsteer.setValue(backRightModule.getSteerAmpDraws());
        // BRdrive.setValue(backRightModule.getDriveAmpDraws());
        
        for (ApriltagWrapper apriltagWrapper : apriltagWrappers) {
            Optional<EstimatedRobotPose> visionEstimate = apriltagWrapper.getRobotPose(
                new Pose3d(field.getRobotPose())
            );

            if (visionEstimate.isPresent()) {
                poseEstimator.addVisionMeasurement(
                    visionEstimate.get().estimatedPose.toPose2d(),
                    visionEstimate.get().timestampSeconds
                );
            }
        }
        
        Rotation2d gyroAngle = getGyroHeading();
        Pose2d estimate = poseEstimator.update(
            gyroAngle,
            getModulePositions()
        );
        
        // if(ahrsTimer.advanceIfElapsed(3)){
        //     System.out.println(gyroAngle);
        //     resetAhrs();
        //     System.out.println(gyroAngle);
        //     System.out.println("RESeT THE AHRS");
        //     ahrsTimer.stop();
        // }

        

        field.setRobotPose(new Pose2d(estimate.getX() + 1, estimate.getY() + .3, estimate.getRotation()));
        
        for (int i = 0; i < 4; i++) {
            angles[i].set(states[i].angle.getRadians());
            velocities[i].set(states[i].speedMetersPerSecond);
        }

        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);

    }

    /** Sets the powers of the drivetrain through PIDs. Relative to the driver heading on the field.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA,
            getDriverHeading()
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /** Sets the power of the drivetrain through PIDs. Relative to the robot with the intake in the front.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setRobotRelativeDrivePowers(double xPower, double yPower, double angularPower) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA, 
            new Rotation2d(0)
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /** Sets the power of the drivetrain through PIDs. Relative to the robot with the intake in the front.
     *
     * @param robotRelativeSpeeds The speeds of the chassis to set to.
     */
    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            new Rotation2d(0)
        );
        // System.out.println(speeds.vxMetersPerSecond);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /** Gets the current chassis speeds relative to the robot.
     *
     * @return The robot relative chassis speeds.
     */
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(states),
            getRobotPosition().getRotation() // getGyroHeading()
        );
        return robotRelativeSpeeds;
    }

    /** Sets the chassis speeds.
     *
     * @param xSpeed x direction speed in meters/second.
     * @param ySpeed y direction speed in meters/second.
     * @param angleSpeed angular speed in rads/second.
     */
    public void setChassisSpeeds(double xSpeed, double ySpeed, double angleSpeed) {
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSpeed,
            ySpeed,
            angleSpeed);
        
        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /** Sets the drive powers with a heading lock. Field relative.
     *
     * @param xPower [-1, 1] The x direction power (forward).
     * @param yPower [-1, 1] The y direction power (left).
     * @param targetAngle Rotation2d of the angle wanted.
     */
    public void setDrivePowersWithHeadingLock(double xPower, double yPower, Rotation2d targetAngle) {
        Rotation2d currentRotation = getRobotPosition().getRotation();
        double turnSpeed = thetaController.calculate(currentRotation.getRadians(), targetAngle.getRadians());
        double turnPower = MathUtil.clamp(turnSpeed / MAX_OMEGA, -1.0, 1.0);

        setDrivePowers(xPower, yPower, turnPower);
    }

    /** Sets the drive powers while aiming at the speaker.
     *
     * @param xPower The power in x direction.
     * @param yPower The power in the y direction.
     */
    public void setSwerveAimDrivePowers(double xPower, double yPower) {
        double shootAngleRadians = getShootAngle(isRed);

        setDrivePowersWithHeadingLock(xPower, yPower, Rotation2d.fromRadians(shootAngleRadians));
    }

    /** Gets the correct angle to aim the swerve at to point at the speaker.
     *
     * @param isRed Whether the team is red or not.
     * @return The angle to point at.
     */
    public double getShootAngle(boolean isRed) {
        double xDistance = getXFromSpeaker(isRed);
        double yDistance = getYFromSpeaker();

        double rawAngle = Math.atan2(yDistance, xDistance) + Math.PI;

        /* atan2() returns a value from -PI to PI, so the angle must be offset by 180 deg if the speaker is in
         the negative x direction (such as when the robot is on the field and aiming at the blue speaker). */
        return rawAngle + ANGLE_OFFSET_FOR_AUTO_AIM; 
    }

    /** Gets the Y distance from the speaker.
     *
     * @return The Y distance from the speaker in meters.
     */
    public double getYFromSpeaker() {
        return BLUE_SPEAKER_POS.getY() - getRobotPosition().getY(); 
    }

    /** Gets the X distance from the speaker.
     *
     * @param isRed Whether the current team is red or not.
     * @return The X distance from the speaker in meters.
     */
    public double getXFromSpeaker(boolean isRed) {
        return getSpeakerPosition(isRed).getX() - getRobotPosition().getX();
    }

    /** Gets the current speaker position.
     *
     * @param isRed If our team is red or not.
     * @return The translation 2d of the speaker.
     */
    public Translation2d getSpeakerPosition(boolean isRed) {
        if (isRed) {
            return RED_SPEAKER_POS;
        } else {
            return BLUE_SPEAKER_POS;
        }
    }


    /** Sets the states of the swerve modules.
     *
     * @param states The array of swerve modules states
     */
    public void setSwerveModuleStates(SwerveModuleState[] states) {
        this.states = states;
    }

    /** Get the module positions.
     *
     * @return The array of module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    /** Get the kinematics object.
     *
     * @return The kinematics object.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** Get the current robot pose.
     *
     * @return The robot Pose2d.
     */
    public Pose2d getRobotPosition() {
        // System.out.println(poseEstimator.getEstimatedPosition().getRotation());
        return poseEstimator.getEstimatedPosition();

    }

    /** Reset the robot pose.
     *
     * @param currentPose The pose to reset to.
     */
    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle, 
            getModulePositions(), 
            currentPose
        );
    }

    /** Reset the pose to 0,0,0. */
    public void resetPose() {
        resetPose(new Pose2d());
    }

    /** Reset the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /** Reset the driver heading to 0. */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }

    /** Get the gyro heading. TODO: uninvert because it is right side up now. */
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    /** Get the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {

        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();
        
        return robotHeading.minus(driverHeadingOffset);
    }

    /** Reset the ahrs on the navX. */
    public void resetAhrs() {
        ahrs.zeroYaw();
    }

    public SequentialCommandGroup choreoSwerveCommand(ChoreoTrajectory traj) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'choreoSwerveCommand'");
    }

}
