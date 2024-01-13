package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveSubsystem extends BaseSwerveSubsystem{
    private final AHRS ahrs;

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

    public SwerveSubsystem() {
        ahrs = new AHRS(SPI.Port.kMXP);

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER);
        
        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);

        inst.startServer();

        choreoTab = Shuffleboard.getTab("Auton");
        field = new Field2d();
        choreoTab.add("Field", field)
        .withPosition(0, 0)
        .withSize(3, 2);

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
    }

    public void periodic() {
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


        this.states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            this.states, speeds,
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
        return Rotation2d.fromDegrees(ahrs.getAngle());
    }

    public Rotation2d getDriverHeading() {

        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();
        
        return robotHeading.minus(driverHeadingOffset);
    }

}
