package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.PhotonWrapper;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.security.GeneralSecurityException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveSubsystem extends BaseSwerveSubsystem{
    private final AHRS ahrs;

    private final Timer crimer;

    public static final double MAX_VEL = 4.90245766303; //calculated
    public static final double MAX_ACCEL = 3; //STUB
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();
    public static final double MAX_ALPHA = 8; //STUB

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private Rotation2d driverHeadingOffset = new Rotation2d();

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;
    private final PhotonWrapper photonWrapper;

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

    private final ShuffleboardTab choreoTab;
    private final Field2d field;

    private final GenericEntry FLsteer, FLdrive, FRsteer, FRdrive, BLsteer, BLdrive, BRsteer, BRdrive;

    public SwerveSubsystem() {
        ahrs = new AHRS(SPI.Port.kMXP);

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET, true);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET, true);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET, true);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET, true);
        
        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

        inst.startServer();

        crimer = new Timer();
        crimer.start();

        choreoTab = Shuffleboard.getTab("Auton");
        field = new Field2d();
        choreoTab.add("Field", field)
        .withPosition(0, 0)
        .withSize(3, 2);

        FLsteer = choreoTab.add("FLsteer", 0.).withPosition(0, 0).getEntry();
        FLdrive = choreoTab.add("FLdrive", 0.).withPosition(0, 1).getEntry();
        
        FRsteer = choreoTab.add("FRsteer", 0.).withPosition(1, 0).getEntry();
        FRdrive = choreoTab.add("FRdrive", 0.).withPosition(1, 1).getEntry();
        
        BLsteer = choreoTab.add("BLsteer", 0.).withPosition(2, 0).getEntry();
        BLdrive = choreoTab.add("BLdrive", 0.).withPosition(2, 1).getEntry();
        
        BRsteer = choreoTab.add("BRsteer", 0.).withPosition(3, 0).getEntry();
        BRdrive = choreoTab.add("BRdrive", 0.).withPosition(3, 1).getEntry();

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

        photonWrapper = new PhotonWrapper(FRONT_CAMERA, FRONT_CAMERA_POSE);
    }

    public void periodic() {
        FLsteer.setValue(frontLeftModule.getSteerAmpDraws());
        FLdrive.setValue(frontLeftModule.getDriveAmpDraws());

        FRsteer.setValue(frontRightModule.getSteerAmpDraws());
        FRdrive.setValue(frontRightModule.getDriveAmpDraws());

        BLsteer.setValue(backLeftModule.getSteerAmpDraws());
        BLdrive.setValue(backLeftModule.getDriveAmpDraws());

        BRsteer.setValue(backRightModule.getSteerAmpDraws());
        BRdrive.setValue(backRightModule.getDriveAmpDraws());
        
        Optional<EstimatedRobotPose> visionEstimate = photonWrapper.getRobotPose(
            new Pose3d(field.getRobotPose())
        );

        if (visionEstimate.isPresent()) poseEstimator.addVisionMeasurement(
            visionEstimate.get().estimatedPose.toPose2d(),
            visionEstimate.get().timestampSeconds
        );

        Rotation2d gyroAngle = getGyroHeading();
        Pose2d estimate = poseEstimator.update(
            gyroAngle,
            getModulePositions()
        );

        field.setRobotPose(estimate);
        
        for(int i = 0; i < 4; i++){
            angles[i].set(states[i].angle.getRadians());
            velocities[i].set(states[i].speedMetersPerSecond);
        }

        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);

    }

    public void setDrivePowers(double xPower, double yPower, double angularPower){
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

    public void setRobotRelativeDrivePowers(double xPower, double yPower, double angularPower){
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

    

    public void setSwerveModuleStates(SwerveModuleState[] states){
        this.states = states;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d currentPose){
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle, 
            getModulePositions(), 
            currentPose
        );
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void resetDriverHeading(Rotation2d currentRotation){
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }

    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());
    }

    public Rotation2d getDriverHeading() {

        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();
        
        return robotHeading.minus(driverHeadingOffset);
    }

}
