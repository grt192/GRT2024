package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.EXTENSION_METERS_PER_ROTATION;
import static frc.robot.Constants.ClimbConstants.LOWER_LIMIT_METERS;
import static frc.robot.Constants.ClimbConstants.RAISE_LIMIT_METERS;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.MotorUtil;

/**
 * Represents a single manually controlled climb arm.
 */
public class ClimbArm {
    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkPIDController extensionPID;
    private final DigitalInput zeroLimitSwitch;

    private static final double EXTENSION_P = 4;
    private static final double EXTENSION_I = 0;
    private static final double EXTENSION_D = 0;
    private static final double EXTENSION_TOLERANCE_METERS = 0.005;

    private boolean isUsingPID;
    private double winchPower;
    private double extensionTarget;

    /**
     * Constructs a new {@link ClimbArm}.
     *
     * @param winchMotorId The motor's CAN ID.
     */
    public ClimbArm(int winchMotorId, int zeroLimitPort, boolean isInverted) {
        /* Configures the motor */
        winchMotor = MotorUtil.createSparkMax(winchMotorId, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(isInverted);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) (RAISE_LIMIT_METERS + .05));
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) (LOWER_LIMIT_METERS - .05));
        });

        this.enableSoftLimits(true);

        extensionEncoder = winchMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_METERS_PER_ROTATION);
        extensionEncoder.setPosition(LOWER_LIMIT_METERS); /* The arm starts lowered. */

        extensionPID = MotorUtil.createSparkPIDController(winchMotor, extensionEncoder);
        extensionPID.setP(EXTENSION_P);
        extensionPID.setI(EXTENSION_I);
        extensionPID.setD(EXTENSION_D);
        isUsingPID = true;

        zeroLimitSwitch = new DigitalInput(zeroLimitPort);
    }

    /** Run this function every periodic loop.*/
    public void update() {
        if (zeroLimitSwitch != null && this.isLimitSwitchPressed()) {
            resetEncoder();
            winchPower = Math.max(0, winchPower);
        }

        if (isUsingPID) {
            extensionPID.setReference(extensionTarget, ControlType.kPosition);
        } else {
            winchMotor.set(winchPower);
        }
    }

    /**
     * Sets this climb arm's speed (has no effect if PID mode is enabled).
     *
     * @param speed The desired speed from -1.0 (downwards) to +1.0 (upwards).
     */
    public void setSpeed(double speed) {
        winchPower = MathUtil.clamp(speed, -1.0, +1.0);
    }

    /**
     * Sets the targeted extension height for this climb arm (has no effect if PID mode is disabled).
     *
     * @param extensionTarget The extension target in meters.
     */
    public void setExtensionTarget(double extensionTarget) {
        this.extensionTarget = MathUtil.clamp(extensionTarget, LOWER_LIMIT_METERS, RAISE_LIMIT_METERS);
    }

    /** Returns this climb arm's current extension in meters. */
    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
    }

    /**
     * Returns true if this climb arm is within tolerance of its target, and false otherwise.
     */
    public boolean isAtExtensionTarget() {
        return Math.abs(this.getCurrentExtension() - extensionTarget) < EXTENSION_TOLERANCE_METERS; 
    }

    /**
     * Enables/disables this climb arm's motor position limits.
     *
     * @param enable True to enable, false to disable.
     */
    public void enableSoftLimits(boolean enable) {
        winchMotor.enableSoftLimit(SoftLimitDirection.kForward, enable);
        // winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, enable);
    }

    /**
     * Enables/disables this climb arm's PID controller.
     *
     * @param enable True to enable, false to disable.
     */
    public void enablePID(boolean enable) {
        isUsingPID = enable;
    }

    /** Returns whether or not this climb arm's limit switch at position 0 is pressed. */ 
    public boolean isLimitSwitchPressed() {
        return !zeroLimitSwitch.get();
    }

    /** Resets the motor's encoder to 0. */
    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}