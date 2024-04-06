package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.GRTUtil;
import frc.robot.util.Pose2dSupplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.ml.neuralnet.Network;

/** Controls motors and functions for the pivot part of shooter mech. */
public class ShooterPivotSubsystem extends SubsystemBase {

    //motors
    private final CANSparkMax pivotMotor;

    //devices
    private RelativeEncoder rotationEncoder;
    private SparkPIDController rotationPIDController;

    //angle PID (CHANGE LATER
    private static final double ANGLE_P = 1;
    private static final double ANGLE_I = 0;
    private static final double ANGLE_D = 0;

    //field
    private boolean autoAim;
    private double currentEncoderAngle;
    private double currentDistance;

    private DoubleSupplier distanceSupplier; //new Pose2d();
    private BooleanSupplier redSupplier;

    private AkimaSplineInterpolator akima;
    private PolynomialSplineFunction angleSpline;

    private final Timer timer = new Timer();

    private double angleOffset = 0;

    private NetworkTableInstance ntInstance;
    private NetworkTable motorsTable;
    private NetworkTableEntry shooter12CurrentEntry;
    private NetworkTableEntry shooter12VoltageEntry;
    private NetworkTableEntry shooter12TemperatureEntry;

    /** Inits motors and pose field. Also inits PID stuff. */
    public ShooterPivotSubsystem(DoubleSupplier distanceSupplier, BooleanSupplier redSupplier) {

        timer.start();
        this.distanceSupplier = distanceSupplier;
        this.redSupplier = redSupplier;

        //motors
        pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_MOTOR_ID, MotorType.kBrushless); 
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);

        //devices
        rotationEncoder = pivotMotor.getEncoder();
        rotationPIDController = pivotMotor.getPIDController();
        rotationPIDController.setOutputRange(-.3, 0.6);


        //setting PID vars
        rotationPIDController.setP(ANGLE_P);
        rotationPIDController.setI(ANGLE_I);
        rotationPIDController.setD(ANGLE_D);
        rotationPIDController.setFF(0); 
        rotationPIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.PID_ERROR_TOLERANCE, 0); 

        //encoder stuff
        rotationEncoder.setPositionConversionFactor(ShooterConstants.CONVERSION_FACTOR);
        rotationEncoder.setVelocityConversionFactor(ShooterConstants.CONVERSION_FACTOR * 60);
        rotationEncoder.setPosition(Units.degreesToRadians(18));
        rotationPIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.PID_ERROR_TOLERANCE, 0); 

        //pivot soft limits
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(65.5));
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(18));
        pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        double[] distances = {ShooterConstants.MIN_SHOOTER_DISTANCE, 
                              2, 
                              3, 
                              4, 
                              5, 
                              6, 
                              ShooterConstants.MAX_SHOOTER_DISTANCE};
        double[] angles = {Units.degreesToRadians(60), 
                           Units.degreesToRadians(52), 
                           Units.degreesToRadians(36), 
                           Units.degreesToRadians(31), //4
                           Units.degreesToRadians(28),
                           Units.degreesToRadians(27.5),
                           Units.degreesToRadians(28.5)};

        // X = distances, Y = angles in rads
        akima = new AkimaSplineInterpolator();
        angleSpline = akima.interpolate(distances, angles);

        //field
        autoAim = false;

        rotationPIDController.setReference(Units.degreesToRadians(18), ControlType.kPosition);
        
        ntInstance = NetworkTableInstance.getDefault();
        motorsTable = ntInstance.getTable("Motors");
        shooter12CurrentEntry = motorsTable.getEntry("Shooter12CurrentEntry");
        shooter12VoltageEntry = motorsTable.getEntry("Shooter12Voltage");
        shooter12TemperatureEntry = motorsTable.getEntry("Shooter12Temperature");
    }
    
    /** motor speed setting functions. */
    public void setPivotMotorSpeed(double speed) {
        pivotMotor.setVoltage(speed * 12);
    }

    /** Sets Angle of the pivot.*/
    public void setAngle(double angle) { 
        rotationPIDController.setReference(angle + angleOffset, CANSparkMax.ControlType.kPosition);
       
    }

    private double getShootingDistance() {
        return distanceSupplier.getAsDouble();
    }

    /** Gets correct Angle for pivot to turn to. */
    public double getAutoAimAngle() {
        
        currentDistance = getShootingDistance();

        // System.out.println("Distance to speaker: " + GRTUtil.twoDecimals(currentDistance) 
        //     + " Set angle: " + GRTUtil.twoDecimals(Units.radiansToDegrees(angleSpline.value(MathUtil.clamp(currentDistance, ShooterConstants.MIN_SHOOTER_DISTANCE, ShooterConstants.MAX_SHOOTER_DISTANCE))))
        //     + " Current angle: " + GRTUtil.twoDecimals(Units.radiansToDegrees(rotationEncoder.getPosition())));

        return angleSpline.value(MathUtil.clamp(getShootingDistance(), ShooterConstants.MIN_SHOOTER_DISTANCE, ShooterConstants.MAX_SHOOTER_DISTANCE));
    }

    /** Gets position of encoder. */
    public double getPosition() {
        return rotationEncoder.getPosition();
    }
    
    /** Sets whether Auto-aim should be on or off. */
    public void setAutoAimBoolean(boolean autonAim) { 
        this.autoAim = autonAim;
    }

    @Override
    public void periodic() {
        if (autoAim) {
            setAngle(getAutoAimAngle());
        }
        
        shooter12CurrentEntry.setDouble(pivotMotor.getOutputCurrent());
        shooter12VoltageEntry.setDouble(pivotMotor.getBusVoltage());
        shooter12TemperatureEntry.setDouble(pivotMotor.getMotorTemperature());
    }
    
    /** Sets the angle offset to a new value.
     *
     * @param angleOffset The value to set the offset to.
     */

    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }
}
