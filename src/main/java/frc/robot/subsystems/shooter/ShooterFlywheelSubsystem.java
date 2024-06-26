// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Pose2dSupplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

/** The flywheel subsystem for the shooter mechanism. */
public class ShooterFlywheelSubsystem extends SubsystemBase {

    //motors
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBottom;

    //devices
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    //Network Tables
    private NetworkTableInstance ntInstance;
    private NetworkTable ntTable;
    private BooleanPublisher ntPublisher;

    private double targetTopRPS;
    private double targetBottomRPS;

    private DoubleSupplier distanceSupplier;
    private BooleanSupplier redSupplier;

    private AkimaSplineInterpolator akima;
    private PolynomialSplineFunction flywheelSpline;
    private PolynomialSplineFunction bottomFlywheelSpline;
    private boolean atSpeed = false;
    private boolean autoAim = false;

    private NetworkTable motorsNTTable;
    private NetworkTableEntry shooter13CurrentEntry;
    private NetworkTableEntry shooter13VoltageEntry;
    private NetworkTableEntry shooter13TemperatureEntry;
    private NetworkTableEntry shooter14CurrentEntry;
    private NetworkTableEntry shooter14VoltageEntry;
    private NetworkTableEntry shooter14TemperatureEntry;
    /**
     * Runs the flywheels for the shooter.
     *
     * @param poseSupplier The poseSupplier for shooter speed calculations.
     */
    public ShooterFlywheelSubsystem(DoubleSupplier distanceSupplier, BooleanSupplier redSupplier) {
        //motors
        shooterMotorTop = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ID);
        shooterMotorBottom = new TalonFX(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID);

        request.EnableFOC = true;

        shooterMotorTop.setInverted(true);
        shooterMotorBottom.setInverted(true);

        shooterMotorTop.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorBottom.setNeutralMode(NeutralModeValue.Coast);

        request.EnableFOC = true;

        Slot0Configs configs = new Slot0Configs();

        double[] distances = {ShooterConstants.MIN_SHOOTER_DISTANCE, 
                              2, 
                              3, 
                              4, 
                              5, 
                              6, 
                              ShooterConstants.MAX_SHOOTER_DISTANCE};

        double[] speeds = {.5, .5, .65, .75, .75, .8, .8};

        targetTopRPS = 0.0;
        targetBottomRPS = 0.0; 

        akima = new AkimaSplineInterpolator();
        flywheelSpline = akima.interpolate(distances, speeds);

        configs.kP = .5;
        configs.kI = 0.05;
        configs.kD = 0;
        configs.kV = .12;

        shooterMotorBottom.getConfigurator().apply(configs);
        shooterMotorTop.getConfigurator().apply(configs);

        this.distanceSupplier = distanceSupplier;
        this.redSupplier = redSupplier;
        //nts
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("RobotStatus");
        ntPublisher = ntTable.getBooleanTopic("shooterReady").publish();
        motorsNTTable = ntInstance.getTable("Motors");
        shooter13CurrentEntry = motorsNTTable.getEntry("Shooter13Current");
        shooter13VoltageEntry = motorsNTTable.getEntry("Shooter13Voltage");
        shooter13TemperatureEntry = motorsNTTable.getEntry("Shooter13Temperature");
        shooter14CurrentEntry = motorsNTTable.getEntry("Shooter14Current");
        shooter14VoltageEntry = motorsNTTable.getEntry("Shooter14Voltage");
        shooter14TemperatureEntry = motorsNTTable.getEntry("Shooter14Temperature");
    }

    /** Sets shooting motor speed.  */
    public void setShooterMotorSpeed(double topSpeed, double bottomSpeed) {
        targetTopRPS = ShooterConstants.MAX_FLYWHEEL_RPS * topSpeed;
        targetBottomRPS = ShooterConstants.MAX_FLYWHEEL_RPS * bottomSpeed;

        shooterMotorTop.setControl(request.withVelocity(targetTopRPS));
        shooterMotorBottom.setControl(request.withVelocity(targetBottomRPS));
        
        atSpeed = Math.abs(targetTopRPS - shooterMotorTop.getVelocity().getValueAsDouble()) < 1
            && Math.abs(targetBottomRPS - shooterMotorBottom.getVelocity().getValueAsDouble()) < 1
            && targetBottomRPS != 0;
        
        // System.out.println("targetTop: " + targetTopRPS + "realTop" + shooterMotorTop.getVelocity().getValueAsDouble());
        // System.out.println("targetBottom: " + targetBottomRPS + "realBottom" + shooterMotorBottom.getVelocity().getValueAsDouble());

    }

    /** Sets shooting motor speed for only one speed. */
    public void setShooterMotorSpeed(double speed) {
        setShooterMotorSpeed(speed, speed);
    }

    /** Sets the shooter to the splined speeds. */
    public void setShooterMotorSpeed() {
        setShooterMotorSpeed(getSplineSpeed(), getSplineSpeed());
    }

    /** Changes auto aim angle. */
    public void setAutoAimShooter(boolean bool) {
        this.autoAim = bool;
    }

    /** Stops the shooter. Better than setting speeds to 0 because this slows them down without a PID. */
    public void stopShooter() {
        shooterMotorTop.set(0);
        shooterMotorBottom.set(0);
        atSpeed = false;
        autoAim = false;
    }

    /**
     * Returns whether the shooter is at the targeted speed or not.
     *
     * @return Whether the shooter is at the targeted speed.
     */
    public boolean atSpeed() {
        return atSpeed;
    }

    /**
     * Returns the current splined top motor speed.
     *
     * @return The splined top motor speed.
     */
    public double getSplineSpeed() {
        return flywheelSpline.value(MathUtil.clamp(
            getShootingDistance(), ShooterConstants.MIN_SHOOTER_DISTANCE, ShooterConstants.MAX_SHOOTER_DISTANCE
        ));
    }

    /**
     * Gets the actual speed of the top shooter motor.
     *
     * @return the actual speed in rotations per second
     */
    public double getTopSpeed() {
        return shooterMotorTop.getVelocity().getValueAsDouble();
    }
    
    /**
     * Gets the actual speed of the bottom shooter motor.
     *
     * @return the actual speed in rotations per second
     */
    public double getBottomSpeed() {
        return shooterMotorBottom.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the setpoint speed of the top shooter motor.
     *
     * @return the target speed in rotations per second
     */
    public double getTargetTopRPS() {
        return targetTopRPS;
    }

    /**
     * Gets the setpoint speed of the bottom shooter motor.
     *
     * @return the target speed in rotations per second
     */
    public double getTargetBottomRPS() {
        return targetBottomRPS;
    }

    public double getShootingDistance() {
        return distanceSupplier.getAsDouble();
    }

    @Override
    public void periodic() {
        //only grabs spline speeds if shooter motor is running (ie not stopped)
        if (autoAim) {
            setShooterMotorSpeed(getSplineSpeed(), getSplineSpeed());
        }

        shooter13CurrentEntry.setDouble(shooterMotorTop.getSupplyCurrent().getValueAsDouble());
        shooter13VoltageEntry.setDouble(shooterMotorTop.getMotorVoltage().getValueAsDouble());
        shooter13TemperatureEntry.setDouble(shooterMotorTop.getDeviceTemp().getValueAsDouble());
        shooter14CurrentEntry.setDouble(shooterMotorBottom.getSupplyCurrent().getValueAsDouble());
        shooter14VoltageEntry.setDouble(shooterMotorBottom.getMotorVoltage().getValueAsDouble());
        shooter14TemperatureEntry.setDouble(shooterMotorBottom.getDeviceTemp().getValueAsDouble());
    }
}