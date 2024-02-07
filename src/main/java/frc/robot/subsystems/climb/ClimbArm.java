package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

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
 * Represents a single Climb arm
 */
public class ClimbArm {
    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkPIDController extensionPIDController;

    private final DigitalInput zeroLimitSwitch;

    private double targetExtension;

    /**
     * Constructs a new {@link ClimbArm}
     * @param winchMotorID The motor's CAN ID
     * @param zeroLimitID The limit switch's RIO port ID
     */
    public ClimbArm(int winchMotorID, int zeroLimitID) {
        /* Configures the motor */
        winchMotor = MotorUtil.createSparkMax(winchMotorID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(EXTENSION_RAMP_RATE);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) RAISE_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) LOWER_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        });

        extensionEncoder = winchMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(1 / ROTATIONS_PER_EXTENSION_METER);
        extensionEncoder.setPosition(0);

        /* Sets up the PID controller */
        extensionPIDController = MotorUtil.createSparkPIDController(winchMotor, extensionEncoder);
        extensionPIDController.setP(EXTENSION_P);
        extensionPIDController.setI(EXTENSION_I);
        extensionPIDController.setD(EXTENSION_D);
        extensionPIDController.setSmartMotionAllowedClosedLoopError(EXTENSION_TOLERANCE_METERS, 0);

        zeroLimitSwitch = new DigitalInput(LEFT_ZERO_LIMIT_ID);

        targetExtension = 0;
    }

    /**
     * Run this function every periodic loop.
     */
    public void update() {
        if (zeroLimitSwitch != null && zeroLimitSwitch.get())
            resetEncoder();

        extensionPIDController.setReference(targetExtension, ControlType.kPosition);

    }
    
    /**
     * Sets the targeted extension height for this climb arm
     * @param targetExtension The extension target
     */
    public void setTargetExtension(double targetExtension) {
        this.targetExtension = MathUtil.clamp(targetExtension, LOWER_LIMIT_METERS, RAISE_LIMIT_METERS);
    }

    /**
     * @return This climb arm's extension target
     */
    public double getTargetExtension() {
        return targetExtension;
    }

    /**
     * @return This climb arm's current extension
     */
    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
    }

    /**
     * @return True if this climb arm is within tolerance of its target, False otherwise
     */
    public boolean isAtTargetExtension() {
        return Math.abs(getCurrentExtension() - getTargetExtension()) < EXTENSION_TOLERANCE_METERS; 
    }

    /**
     * Resets the motor's encoder to 0
     */
    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}
