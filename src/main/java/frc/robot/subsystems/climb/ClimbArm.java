package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.EXTENSION_METERS_PER_ROTATION;
import static frc.robot.Constants.ClimbConstants.LOWER_LIMIT_METERS;
import static frc.robot.Constants.ClimbConstants.RAISE_LIMIT_METERS;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.MotorUtil;

/**
 * Represents a single manually controlled climb arm.
 */
public class ClimbArm {
    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;
    private final DigitalInput zeroLimitSwitch;

    private double winchPower;
    private double prevWinchPower;

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

        zeroLimitSwitch = new DigitalInput(zeroLimitPort);

        winchPower = 0;
        prevWinchPower = 0;
    }

    /** Run this function every periodic loop.*/
    public void update(double speed) {
        winchPower = MathUtil.clamp(speed, -1.0, +1.0);

        if (this.isLimitSwitchPressed()) {
            resetEncoder();
            winchPower = Math.max(-.1, winchPower);
        }

        if (winchPower != prevWinchPower) {
            winchMotor.set(winchPower);
        }

        prevWinchPower = winchPower;
    }

    /**
     * Sets this climb arm's speed (has no effect if PID mode is enabled).
     *
     * @param speed The desired speed from -1.0 (downwards) to +1.0 (upwards).
     */
    public void setSpeed(double speed) {
        winchPower = MathUtil.clamp(speed, -1.0, +1.0);
    }

    /** Returns this climb arm's current extension in meters. */
    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
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

    /** Returns whether or not this climb arm's limit switch at position 0 is pressed. */ 
    public boolean isLimitSwitchPressed() {
        return !zeroLimitSwitch.get();
    }

    /** Resets the motor's encoder to 0. */
    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}