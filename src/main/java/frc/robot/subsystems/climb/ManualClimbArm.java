package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.EXTENSION_METERS_PER_ROTATION;
import static frc.robot.Constants.ClimbConstants.LOWER_LIMIT_METERS;
import static frc.robot.Constants.ClimbConstants.RAISE_LIMIT_METERS;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.MotorUtil;

/**
 * Represents a single manually controlled Climb arm.
 */
public class ManualClimbArm {
    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;

    private double winchPower;

    /**
     * Constructs a new {@link ClimbArm}.
     *
     * @param winchMotorId The motor's CAN ID.
     */
    public ManualClimbArm(int winchMotorId, boolean isInverted) {
        /* Configures the motor */
        winchMotor = MotorUtil.createSparkMax(winchMotorId, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(isInverted);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) RAISE_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) LOWER_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        });

        extensionEncoder = winchMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_METERS_PER_ROTATION);
        extensionEncoder.setPosition(RAISE_LIMIT_METERS); /* The arm starts extended */
    }

    /**
     * Run this function every periodic loop.
     */
    public void update() {
        winchMotor.set(winchPower);
    }

    /**
     * Sets this climb arm's speed.
     *
     * @param speed The desired speed from -1.0 (downwards) to +1.0 (upwards).
     */
    public void setSpeed(double speed) {
        winchPower = MathUtil.clamp(speed, -1.0, +1.0);
    }

    /**
     * Returns this climb arm's current extension in meters.
     */
    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
    }
}