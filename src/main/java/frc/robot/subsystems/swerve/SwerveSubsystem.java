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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.GRTUtil;
import frc.robot.vision.ApriltagWrapper;

import java.math.MathContext;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;

/** The subsystem that controls the swerve drivetrain. */
public class SwerveSubsystem extends SubsystemBase {
    private final AHRS ahrs;

    private final Timer lockTimer = new Timer();
    private static final double LOCK_TIMEOUT_SECONDS = 1.0; // The elapsed idle time to wait before locking

    public static final double MAX_VEL = 4.172; //calculated
    public static final double MAX_ACCEL = 3;
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();
    public static final double MAX_ALPHA = 8;

    public static final double ANGLE_OFFSET_FOR_AUTO_AIM = Units.degreesToRadians(0);
    public static final double SHOT_SPEED = 30; // meters per sec

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

    /* Heading Lock Controller */
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
    private final Field2d fieldVisualization;

    private final GenericEntry frontLeftSteer;
    private final GenericEntry frontRightSteer;
    private final GenericEntry backLeftSteer;
    private final GenericEntry backRightSteer;
    private final GenericEntry robotPosEntry;

    private boolean verbose = false;

    private BooleanSupplier redSupplier;

    private Translation2d targetPoint = new Translation2d();
    private boolean aiming = false;
    private Pose2d lastPose = new Pose2d();
    private Pose2d velocity = new Pose2d();
    private Timer velocityTimer = new Timer();

    private NetworkTableEntry xEntry = networkTable.getEntry("x");
    private NetworkTableEntry yEntry = networkTable.getEntry("y");
    /** Constructs a {@link SwerveSubsystem}. */
    public SwerveSubsystem(BooleanSupplier redSupplier) {
        ahrs = new AHRS(SPI.Port.kMXP);

        this.redSupplier = redSupplier;

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET, true);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET, true);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET, true);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET, true);
        
        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

        thetaController = new PIDController(4, 0, 0);
        thetaController.enableContinuousInput(Math.PI, -Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(3));

        inst.startServer();

        choreoTab = Shuffleboard.getTab("Auton");
        fieldVisualization = new Field2d();
        choreoTab.add("Field", fieldVisualization)
            .withPosition(0, 0)
            .withSize(6, 4);

        robotPosEntry = choreoTab.add("position", 0.).withPosition(7, 0).getEntry();

        frontLeftSteer = choreoTab.add("FLsteer", 0.).withPosition(0, 0).getEntry();
        frontRightSteer = choreoTab.add("FRsteer", 0.).withPosition(1, 0).getEntry();
        backLeftSteer = choreoTab.add("BLsteer", 0.).withPosition(2, 0).getEntry();     
        backRightSteer = choreoTab.add("BRsteer", 0.).withPosition(3, 0).getEntry();

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
            redSupplier,
            this
        );

        velocityTimer.start();
    }

    @Override
    public void periodic() {

        robotPosEntry.setValue(GRTUtil.twoDecimals(getRobotPosition().getX()));
        
        for (ApriltagWrapper apriltagWrapper : apriltagWrappers) {
            Optional<EstimatedRobotPose> visionEstimate = apriltagWrapper.getRobotPose(
                new Pose3d(getRobotPosition())
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
    

        fieldVisualization.setRobotPose(new Pose2d(
            GRTUtil.twoDecimals(estimate.getX() + 1),
            estimate.getY() + .3,
            estimate.getRotation())
        );
        
        // If all commanded velocities are 0, the system is idle (drivers / commands are
        // not supplying input).
        boolean isIdle = states[0].speedMetersPerSecond == 0.0
            && states[1].speedMetersPerSecond == 0.0
            && states[2].speedMetersPerSecond == 0.0
            && states[3].speedMetersPerSecond == 0.0;

        // Start lock timer when idle
        if (isIdle) {
            lockTimer.start();
        } else {
            lockTimer.stop();
            lockTimer.reset();
        }

        // Lock the swerve module if the lock timeout has elapsed, or set them to their 
        // setpoints if drivers are supplying non-idle input.
        if (lockTimer.hasElapsed(LOCK_TIMEOUT_SECONDS)) {
            applyLock();
        } else {
            for (int i = 0; i < 4; i++) {
                angles[i].set(states[i].angle.getRadians());
                velocities[i].set(states[i].speedMetersPerSecond);
            }

            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);   
        }

        if (verbose) {
            printModuleAngles();
        }

        //update velocity
        if (velocityTimer.hasElapsed(.1)) {
            double timePassed = velocityTimer.get();
            velocityTimer.restart();
            double vX = (getRobotPosition().getX() - lastPose.getX()) / timePassed;
            double vY = (getRobotPosition().getY() - lastPose.getY()) / timePassed;
            double vTheta = (getRobotPosition().getRotation().getRadians() - lastPose.getRotation().getRadians()) 
                / timePassed;
            velocity = new Pose2d(new Translation2d(vX, vY), new Rotation2d(vTheta));
            lastPose = getRobotPosition();

            // System.out.println("shooting pos: " + getShootingPosition());
        }
        xEntry.setDouble(getShootingPosition().getX());
        yEntry.setDouble(getShootingPosition().getY());
    }

    /**
     * Sets the powers of the drivetrain through PIDs. Relative to the driver heading on the field.
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

        speeds = getAimChassisSpeeds(speeds);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /**
     * Sets the power of the drivetrain through PIDs. Relative to the robot with the intake in the front.
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

        speeds = getAimChassisSpeeds(speeds);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /**
     * Sets the power of the drivetrain through PIDs. Relative to the robot with the intake in the front.
     *
     * @param robotRelativeSpeeds The speeds of the chassis to set to.
     */
    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            new Rotation2d(0)
        );

        speeds = getAimChassisSpeeds(speeds);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }

    /**
     * Sets the chassis speeds.
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
        
        speeds = getAimChassisSpeeds(speeds);
        
        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }
   

    /**
     * Sets the drive powers with a heading lock. Field relative.
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

    /**
     * Sets the states of the swerve modules.
     *
     * @param states The array of swerve modules states
     */
    public void setSwerveModuleStates(SwerveModuleState[] states) {
        this.states = states;
    }

    /** Executes swerve X locking, putting swerve's wheels into an X configuration to prevent motion.
     */
    public void applyLock() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)));
        backRightModule.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)));
    }

    /**
     * Gets the current chassis speeds relative to the robot.
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

    /**
     * Gets the module positions.
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

    public void setAim(boolean aiming){
        this.aiming = aiming;
    }

    public void setTargetPoint(Translation2d targetPoint) {
        this.targetPoint = targetPoint;
    }

    public void targetSpeaker(){
        targetPoint = redSupplier.getAsBoolean() ? SwerveConstants.RED_SPEAKER_POS : SwerveConstants.BLUE_SPEAKER_POS;
    }

    public ChassisSpeeds getAimChassisSpeeds(ChassisSpeeds currentSpeeds){
        if (aiming) {
            Rotation2d currentRotation = getRobotPosition().getRotation();
            currentSpeeds.omegaRadiansPerSecond = 
                thetaController.calculate(currentRotation.getRadians(), getShootingAngle()) 
                * MathUtil.clamp(MathUtil.applyDeadband(velocity.getTranslation().getNorm(), .2) * 1 + 1, 1, 3);
        }
        return currentSpeeds;
    }

    public boolean atTargetAngle(){
        return Math.abs(getAngleError()) < Units.degreesToRadians(5);
    }

    public Translation2d getTargetPoint() {
        return targetPoint;
    }

    private double getXFromTarget() {
        return targetPoint.getX() - getRobotPosition().getX();
    }

    private double getYFromTarget() {
        return targetPoint.getY() - getRobotPosition().getY();
    }

    private double getDistanceFromTarget() {
        return Math.sqrt(getXFromTarget() * getXFromTarget() + getYFromTarget() * getYFromTarget());
    }

    /** Gets pivot angle to any point on the field. */
    public double getAngleToTarget() {
        double xDist = getXFromTarget();
        double yDist = getYFromTarget();

        return Math.atan2(yDist, xDist) + Math.PI;
    }

    public double getShootingAngle() {
        Translation2d shootingPos = getShootingPosition();
        double xDist = getTargetPoint().getX() - shootingPos.getX();
        double yDist = getTargetPoint().getY() - shootingPos.getY();

        return Math.atan2(yDist, xDist) + Math.PI;
    }

    /** Returns the PID error for the rotation controller. */
    public double getAngleError() {
        return thetaController.getPositionError();
    }

    /**
     * Gets the kinematics object.
     *
     * @return The kinematics object.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Gets the current robot pose.
     *
     * @return The robot Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getRobotVelocity() {
        return velocity;
    }

    public Translation2d getShootingPosition() {
        double shotTime = getDistanceFromTarget() / SHOT_SPEED;
        Pose2d robotPos = getRobotPosition();
        Translation2d estimatedPose = new Translation2d();
        for (int i = 0; i < 3; i++) {
            double estimatedX = getRobotVelocity().getX() * shotTime + robotPos.getX();
            double estimatedY = getRobotVelocity().getY() * shotTime + robotPos.getY();
            estimatedPose = new Translation2d(estimatedX, estimatedY);
            shotTime = estimatedPose.getDistance(getTargetPoint()) / SHOT_SPEED;
        }
        return estimatedPose;
    }

    public double getShootingDistance() {
        return getShootingPosition().getDistance(targetPoint);
    }

    /**
     * Resets the robot pose.
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

    /** Resets the pose to 0,0,0. */
    public void resetPose() {
        resetPose(new Pose2d());
    }

    /**
     * Resets the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /** Resets the driver heading to 0. */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }

    /** Gets the gyro heading.*/
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    /**
     * Gets the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {

        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();
        
        return robotHeading.minus(driverHeadingOffset);
    }

    /** Resets the ahrs on the navX. */
    public void resetAhrs() {
        ahrs.zeroYaw();
    }

    /** Prints the current module angles. Used for zeroing swerve. */
    public void printModuleAngles() {
        System.out.println("FL: " + GRTUtil.twoDecimals(frontLeftModule.getRawAngle().getRadians())
                        + " FR: " + GRTUtil.twoDecimals(frontRightModule.getRawAngle().getRadians())
                        + " BL: " + GRTUtil.twoDecimals(backLeftModule.getRawAngle().getRadians())
                        + " BR: " + GRTUtil.twoDecimals(backRightModule.getRawAngle().getRadians()));
    }

    /**
     * Sets the verbosity of the swerve modules. Verbose mode prints out the angles of every module.
     *
     * @param isVerbose Whether to print the module angles.
     */
    public void setVerbose(boolean isVerbose) {
        verbose = isVerbose;
    }
}