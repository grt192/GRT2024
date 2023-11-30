package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.MotorUtil;

public class SwerveModule {
    private final SwerveDriveMotor driveMotor;

    private final CANSparkMax steerMotor;
    private RelativeEncoder steerRelativeEncoder;
    private SparkMaxAnalogSensor steerAbsoluteEncoder;
    private SparkMaxPIDController steerPidController;

    private double offsetRads;

    private static final double STEER_VOLTS_RADIANS = 1; //STUB

    public SwerveModule(int drivePort, int steerPort, double offsetRads, boolean falcon) {
        
        driveMotor = falcon ? new FalconDriveMotor(drivePort) : new NEODriveMotor(drivePort);
        
        steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);

        steerAbsoluteEncoder = steerMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_RADIANS);

        steerPidController = MotorUtil.createSparkMaxPIDController(steerMotor, steerAbsoluteEncoder);
        steerPidController.setP(0);
        steerPidController.setI(0);
        steerPidController.setD(0);
        steerPidController.setFF(0);

        steerPidController.setPositionPIDWrappingEnabled(true);
        steerPidController.setPositionPIDWrappingMinInput(0.0);
        steerPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        this.offsetRads = offsetRads;
    }

    /**
     * Default to Falcon motor
     * @param drivePort
     * @param steerPort
     * @param offsetRads
     */
    public SwerveModule(int drivePort, int steerPort, double offsetRads){
        this(drivePort, steerPort, offsetRads, true);
    }

    /**
     * Default to no offset rads
     * @param drivePort
     * @param steerPort
     */
    public SwerveModule(int drivePort, int steerPort){
        this(drivePort, steerPort, 0.0, true);
    }

    public SwerveModulePosition getState(){
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle()
        );
    }

    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);

        double targetAngleRads = optimized.angle.getRadians() - offsetRads;
        double angleErrorRads = optimized.angle.minus(currentAngle).getRadians();

        double targetVelocity = optimized.speedMetersPerSecond * Math.abs(Math.cos(angleErrorRads));
        double currentVelocity = driveMotor.getVelocity();

        driveMotor.setVelocity(targetVelocity);
        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
    }

    public void setRawPowers(double drivePower, double steerPower){
        driveMotor.setPower(drivePower);
        steerMotor.set(steerPower);
    }

    public void setRawPowersWithAngle(double drivePower, double angleRads){        
        driveMotor.setPower(drivePower);
        steerPidController.setReference(angleRads, ControlType.kPosition);
    }

    private Rotation2d getWrappedAngle(){
        double angleRads = steerAbsoluteEncoder.getPosition();
        double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(wrappedAngleRads);
    }
}
