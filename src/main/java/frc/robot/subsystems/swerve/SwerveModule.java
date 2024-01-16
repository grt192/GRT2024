package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

public class SwerveModule {
    private final SwerveDriveMotor driveMotor;

    private final CANSparkMax steerMotor;
    private RelativeEncoder steerRelativeEncoder;
    private SparkAnalogSensor steerAbsoluteEncoder;
    private SparkPIDController steerPidController;

    private double offsetRads;

    private boolean verbose;

    
    private static final double DRIVE_METERS_PER_ROTATION = (13.0 / 90.0) * Math.PI * Units.inchesToMeters(4.0);
    private static final double DRIVE_ROTATIONS_PER_METER = 1.0 / DRIVE_METERS_PER_ROTATION;
    private static final double STEER_ROTATIONS_PER_RADIAN = (130.0 / 1776.0) * 2.0 * Math.PI; // Useful for steer relative encoder if we ever use that
    private static final double STEER_VOLTS_RADIANS = 2 * Math.PI / 3.3 ; // https://docs.revrobotics.com/sparkmax/feature-description/data-port#analog-input
    //The encoder board maps the 5V output of the encoder to 3.3V of the Spark Max

    // TODO: tune PIDs, comments are 2023 constants
    private static final double FALCON_DRIVE_P = 0.0028; // .05
    private static final double FALCON_DRIVE_I = 0; // 0
    private static final double FALCON_DRIVE_D = 0.005; // 0
    private static final double FALCON_DRIVE_FF = .11285266; //  .11285266;

    private static final double VORTEX_DRIVE_P = 0;
    private static final double VORTEX_DRIVE_I = 0;
    private static final double VORTEX_DRIVE_D = 0;
    private static final double VORTEX_DRIVE_FF = 0;

    private static final double STEER_P = .68; // .7
    private static final double STEER_I = 0; // 0
    private static final double STEER_D = 0; // 35
    private static final double STEER_FF = 0; // 0

    private Timer crimor;

    /**
     * Constructs a Swerve Module
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder
     * @param falcon Whether this is a falcon or not
     */

    public SwerveModule(int drivePort, int steerPort, double offsetRads, boolean falcon) {
        
        driveMotor = falcon ? new FalconDriveMotor(drivePort) : new VortexDriveMotor(drivePort);
        
        if(falcon){
            driveMotor.configPID(FALCON_DRIVE_P, FALCON_DRIVE_I, FALCON_DRIVE_D, FALCON_DRIVE_FF);
        } else {
            driveMotor.configPID(VORTEX_DRIVE_P, VORTEX_DRIVE_I, VORTEX_DRIVE_D, VORTEX_DRIVE_FF);
        }

        // untested for vortexes
        driveMotor.setPositionConversionFactor(DRIVE_ROTATIONS_PER_METER);
        System.out.println("factor " + DRIVE_ROTATIONS_PER_METER);
        driveMotor.setVelocityConversionFactor(DRIVE_ROTATIONS_PER_METER * 60.0); //Conversion from rpm to m/s
        
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
        steerPidController.setPositionPIDWrappingMaxInput( 2 * Math.PI);

        crimor = new Timer();
        crimor.start();


        this.offsetRads = offsetRads;
    }

    /**
     * Defaults to Falcon motor
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads THe offset of the absolute encoder
     */
    public SwerveModule(int drivePort, int steerPort, double offsetRads){
        this(drivePort, steerPort, offsetRads, true);
    }

    /**
     * Defaults to no offset rads
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     */
    public SwerveModule(int drivePort, int steerPort){
        this(drivePort, steerPort, 0.0, true);
    }

    /**
     * Gets the current state of the swerve module
     * @return The current SwerveModulePosition of this module
     */
    public SwerveModulePosition getState(){
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle()
        );
    }

    /**
     * Sets the desired state of this swerve module
     * Does this through setting the PID targets
     * @param state The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);

        double targetAngleRads = optimized.angle.getRadians() - offsetRads;
        double angleErrorRads = optimized.angle.minus(currentAngle).getRadians();

        double targetVelocity = optimized.speedMetersPerSecond; //* Math.abs(Math.cos(angleErrorRads));
        // double currentVelocity = driveMotor.getVelocity();

        driveMotor.setVelocity(targetVelocity);

        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
    }

    public void setVerbose(){
        if (crimor.advanceIfElapsed(.1)){
            // System.out.print(" current " + twoDecimals(getWrappedAngle().getDegrees()));
            // System.out.println(" target " + twoDecimals(Math.toDegrees(MathUtil.angleModulus(targetAngleRads))));
            System.out.print(" error " + twoDecimals(driveMotor.getError()));
            System.out.println(" target " + twoDecimals(driveMotor.getSetPoint()));
        }
    }

    /**
     * Sets the raw powers of the swerve module
     * @param drivePower The power for the drive motor
     * @param steerPower The power for the steer motor
     */
    public void setRawPowers(double drivePower, double steerPower){
        driveMotor.setPower(drivePower);
        steerMotor.set(steerPower);
    }

    /**
     * Sets the raw power for the drive motor, with PID reference for the steer motor
     * @param drivePower The power for the drive motor
     * @param angleRads The requested angle for the steer motor
     */
    public void setRawPowersWithAngle(double drivePower, double angleRads){
                
        Rotation2d currentAngle = getWrappedAngle();
        // SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(0, new Rotation2d(angleRads)), currentAngle);

        double targetAngleRads = angleRads - offsetRads;
        
        driveMotor.setPower(drivePower);
        System.out.println(Math.abs(currentAngle.minus(new Rotation2d(targetAngleRads)).getDegrees()));
        //System.out.print("target " + new Rotation2d(targetAngleRads).getDegrees()  + "--------");
        // System.out.print("error " + (angleRads.minus(currentAngle).getRadians()) + "--------");

        steerPidController.setReference(targetAngleRads, ControlType.kPosition);
        // System.out.println("2");
    }

    /**
     * Gets the current angle of the module
     * @return Wrapped angle in radians from -pi to pi
     */
    public Rotation2d getWrappedAngle(){
        double angleRads = steerAbsoluteEncoder.getPosition();
        double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(wrappedAngleRads);
    }

    public double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

    public double getDriveAmpDraws(){
        return driveMotor.getAmpDraw();
    }

    public double getSteerAmpDraws(){
        return steerMotor.getOutputCurrent();
    }
}
