package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.drivemotors.FalconDriveMotor;
import frc.robot.subsystems.swerve.drivemotors.SwerveDriveMotor;
import frc.robot.subsystems.swerve.drivemotors.VortexDriveMotor;
import frc.robot.util.MotorUtil;

/** One module of swerve. Swerve is a set of 4 modules which each contain one wheel that has independent control
 *  over velocity and angle.
 */
public class SwerveModule {
    private final SwerveDriveMotor driveMotor;

    private final CANSparkMax steerMotor;
    // private RelativeEncoder steerRelativeEncoder;
    private SparkAnalogSensor steerAbsoluteEncoder;
    private SparkPIDController steerPidController;

    private double offsetRads;
    
    // private static final double DRIVE_METERS_PER_ROTATION = (13.0 / 90.0) * Math.PI * Units.inchesToMeters(4.0);
    private static final double DRIVE_ROTATIONS_PER_METER = 4.172 / 6428 * 60 * 4 / 3.42 * 3 / 3.08;
    // private static final double STEER_ROTATIONS_PER_RADIAN = (130.0 / 1776.0) * 2.0 * Math.PI;
    private static final double STEER_VOLTS_RADIANS = 2 * Math.PI / 3.3; 
    // https://docs.revrobotics.com/sparkmax/feature-description/data-port#analog-input
    //The encoder board maps the 5V output of the encoder to 3.3V of the Spark Max

    private static final double MAX_VEL = 4.172;
    private static final double FALCON_DRIVE_P = 0.0028;
    private static final double FALCON_DRIVE_I = 0;
    private static final double FALCON_DRIVE_D = 0.005;
    private static final double FALCON_DRIVE_FF = .11285266; 

    private static final double VORTEX_DRIVE_P = .35;
    private static final double VORTEX_DRIVE_I = 0;
    private static final double VORTEX_DRIVE_D = 0;
    private static final double VORTEX_DRIVE_FF = DRIVE_ROTATIONS_PER_METER  * MAX_VEL * 1.47566;

    private static final double STEER_P = .68;
    private static final double STEER_I = 0;
    private static final double STEER_D = 0;
    private static final double STEER_FF = 0;

    private Timer crimor;

    private double maxEncoder = Units.degreesToRadians(360);
    private double minEncoder = 0;

    /** Constructs a Swerve Module.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder
     * @param vortex Whether this is a falcon or not
     */

    public SwerveModule(int drivePort, int steerPort, double offsetRads, boolean vortex) {
        
        driveMotor = vortex ? new VortexDriveMotor(drivePort) : new FalconDriveMotor(drivePort);
        
        if (vortex) {
            driveMotor.configPID(VORTEX_DRIVE_P, VORTEX_DRIVE_I, VORTEX_DRIVE_D, VORTEX_DRIVE_FF);
        } else {
            driveMotor.configPID(FALCON_DRIVE_P, FALCON_DRIVE_I, FALCON_DRIVE_D, FALCON_DRIVE_FF);
        }

        // untested for vortexes
        driveMotor.setPositionConversionFactor(DRIVE_ROTATIONS_PER_METER);
        driveMotor.setVelocityConversionFactor(DRIVE_ROTATIONS_PER_METER / 60.0); //Conversion from rpm to m/s
        
        steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);
        // steerMotor.setInverted(true);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerAbsoluteEncoder = steerMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        steerAbsoluteEncoder.setPositionConversionFactor(STEER_VOLTS_RADIANS);
        steerAbsoluteEncoder.setInverted(false);
        steerPidController = MotorUtil.createSparkPIDController(steerMotor, steerAbsoluteEncoder);
        steerPidController.setP(STEER_P);
        steerPidController.setI(STEER_I);
        steerPidController.setD(STEER_D);
        steerPidController.setFF(STEER_FF);

        steerPidController.setPositionPIDWrappingEnabled(true);
        steerPidController.setPositionPIDWrappingMinInput(0);
        steerPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        crimor = new Timer();
        crimor.start();

        this.offsetRads = offsetRads;

    }

    /** Defaults to Falcon motor.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads THe offset of the absolute encoder
     */
    public SwerveModule(int drivePort, int steerPort, double offsetRads) {
        this(drivePort, steerPort, offsetRads, true);
    }

    /** Defaults to no offset rads.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     */
    public SwerveModule(int drivePort, int steerPort) {
        this(drivePort, steerPort, 0.0, true);
    }

    /** Gets the current state of the swerve module.
     *
     * @return The current SwerveModulePosition of this module
     */
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle()
        );
    }

    /** Sets the desired state of this swerve module through setting the PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);

        double targetAngleRads = optimized.angle.getRadians() - offsetRads;
        double angleErrorRads = optimized.angle.minus(currentAngle).getRadians();

        // Multiply by cos so we don't move quickly when the swerves are angled wrong
        double targetVelocity = optimized.speedMetersPerSecond * Math.cos(angleErrorRads);

        driveMotor.setVelocity(targetVelocity);

        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
    }

    /** Sets the raw powers of the swerve module.
     *
     * @param drivePower The power for the drive motor
     * @param steerPower The power for the steer motor
     */
    public void setRawPowers(double drivePower, double steerPower) {
        driveMotor.setPower(drivePower);
        steerMotor.set(steerPower);
    }

    /** Sets the raw power for the drive motor, with PID reference for the steer motor.
     *
     * @param drivePower The power for the drive motor
     * @param angleRads The requested angle for the steer motor
     */
    public void setRawPowersWithAngle(double drivePower, double angleRads) {
                
        // Rotation2d currentAngle = getWrappedAngle();
        // SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(
        //    0, new Rotation2d(angleRads)), currentAngle);

        double targetAngleRads = angleRads - offsetRads;
        driveMotor.setPower(drivePower);
        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
    }

    /** Sets the config for the modules encoder. Basically maps the output values to these real values.
     *
     * @param min The minimum value that the encoder outputs in radians
     * @param max The maximum value that the encoder outputs in radians
     */
    public void configureEncoder(double min, double max) {
        minEncoder = min;
        maxEncoder = max;

    }

    /** Gets the angle of the steer after being mapped through encoder correction.
     *
     * @return [0, 2pi] The mapped angle in radians.
     */
    public double getMappedAngle() {
        return 2 * Math.PI * (steerAbsoluteEncoder.getPosition() - minEncoder) / ((double) (maxEncoder - minEncoder));
    }

    /** Gets the current angle of the module.
     *
     * @return Wrapped angle in radians from -pi to pi
     */
    public Rotation2d getWrappedAngle() {
        double angleRads = getMappedAngle();
        double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(wrappedAngleRads);
    }

    /** Gets the raw angle of the encoder.
     *
     * @return Rotation2d of the raw angle of the steer. 
     */
    public Rotation2d getRawAngle() {
        return new Rotation2d(steerAbsoluteEncoder.getPosition());
    }

    /** Gets the error of the drive motor.
     *
     * @return The velocity PID error of the drive motor.
     */
    public double getDriveError() {
        return driveMotor.getError();
    }

    /** Gets the setpoint of the drive motor.
     *
     * @return The current setpoint of the drive motor.
     */
    public double getDriveSetpoint() {
        return driveMotor.getSetpoint();
    }

    /** Gets the current amp draw of the drive motor.
     *
     * @return The amp draw of the drive motor.
     */
    public double getDriveAmpDraw() {
        return driveMotor.getAmpDraw();
    }

    /** Gets the current amp draw of the steer motor.
     *
     * @return The current amp draw of the steer motor.
     */
    public double getSteerAmpDraws() {
        return steerMotor.getOutputCurrent();
    }
    
    /** Gets the distance the distance driven by the drive motor.
     *
     * @return The distance driven in meters.
     */
    public double getDistanceDriven() {
        return driveMotor.getDistance();
    }

    /** Gets the velocity of the drive motor.
     *
     * @return The velocity of the drive motor in meters/second.
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }
}
