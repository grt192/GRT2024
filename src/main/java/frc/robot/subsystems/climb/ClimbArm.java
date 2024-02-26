package frc.robot.subsystems.climb;

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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.util.MotorUtil;

/**
 * Represents a single Climb arm.
 */
public class ClimbArm {
    private static final double EXTENSION_TOLERANCE_METERS = 0.01;
    private static final double EXTENSION_RAMP_RATE = 0.5;
    private static final double EXTENSION_METERS_PER_ROTATION = 1 / 224.55; /* Find through empirical testing. */

    private static final double EXTENSION_P = 0;
    private static final double EXTENSION_I = 0;
    private static final double EXTENSION_D = 0;

    private static final double ZEROING_SPEED = 0.3;

    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkPIDController extensionPidController;

    private final DigitalInput zeroLimitSwitch;
    private final Solenoid latch;

    private double targetExtension;
    private boolean hookIsLatched;
    private boolean isZeroing;

    /**
     * Constructs a new {@link ClimbArm}.
     *
     * @param winchMotorId The motor's CAN ID.
     * @param zeroLimitPort The limit switch's RIO port.
     * @param latchPort The solenoid's PCM port.
     */
    public ClimbArm(int winchMotorId, int zeroLimitPort, int latchPort) {
        /* Configures the motor */
        winchMotor = MotorUtil.createSparkMax(winchMotorId, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(EXTENSION_RAMP_RATE);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) RAISE_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) LOWER_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        });

        extensionEncoder = winchMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(EXTENSION_METERS_PER_ROTATION);
        extensionEncoder.setPosition(0);

        /* Sets up the PID controller */
        extensionPidController = MotorUtil.createSparkPIDController(winchMotor, extensionEncoder);
        extensionPidController.setP(EXTENSION_P);
        extensionPidController.setI(EXTENSION_I);
        extensionPidController.setD(EXTENSION_D);
        extensionPidController.setSmartMotionAllowedClosedLoopError(EXTENSION_TOLERANCE_METERS, 0);

        zeroLimitSwitch = new DigitalInput(zeroLimitPort);
        Solenoid tempLatch;
        try {
            tempLatch = new Solenoid(PneumaticsModuleType.CTREPCM, latchPort);
            hookIsLatched = true;
        } catch (Exception e) {
            tempLatch = null;
            hookIsLatched = false;
            System.out.println("Climb arm initialized without a solenoid latch.");
        }
        latch = tempLatch;

        targetExtension = 0;
    }

    /**
     * Run this function every periodic loop.
     */
    public void update() {
        if (zeroLimitSwitch != null && this.isLimitSwitchPressed()) {
            resetEncoder();
        }

        if (isZeroing && !this.isLimitSwitchPressed()) {
            winchMotor.set(-ZEROING_SPEED);
        } else if (isZeroing && this.isLimitSwitchPressed()) {
            winchMotor.set(0);
        }
        
        /* If the hook is latched in place, the motor can be safely un-powered to avoid stalling.*/
        if (hookIsLatched && !isZeroing) {
            winchMotor.set(0);
        } else if (!isZeroing){
            extensionPidController.setReference(targetExtension, ControlType.kPosition);
        }
    }
    
    /**
     * Sets the targeted extension height for this climb arm.
     *
     * @param targetExtension The extension target in meters.
     */
    public void setTargetExtension(double targetExtension) {
        this.targetExtension = MathUtil.clamp(targetExtension, LOWER_LIMIT_METERS, RAISE_LIMIT_METERS);
    }

    /**
     * Returns this climb arm's PID extension target in meters.
     */
    public double getTargetExtension() {
        return targetExtension;
    }

    /**
     * Returns this climb arm's current extension in meters.
     */
    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
    }

    /**
     * Returns true if this climb arm is within tolerance of its target, and false otherwise.
     */
    public boolean isAtTargetExtension() {
        return Math.abs(this.getCurrentExtension() - this.getTargetExtension()) < EXTENSION_TOLERANCE_METERS; 
    }

    /**
     * Closes the solenoid latch.
     */
    public void closeLatch() {
        if (latch == null) {
            return;
        }

        latch.set(false);

        if (this.getCurrentExtension() < LOWER_LIMIT_METERS + EXTENSION_TOLERANCE_METERS) {
            hookIsLatched = true;
        }
    }

    /**
     * Opens the solenoid latch.
     *
     * @implNote Keeping it open uses ~2.3 A at 12 V per arm, so it is recommended to keep it closed whenever possible.
     */
    public void openLatch() {
        if (latch == null) {
            return;
        }

        hookIsLatched = false;
        latch.set(true);
    }

    /**
     * Enables/disables this Climb arm's zeroing mode.
     * 
     * @param enable Enables if true, disables if false..
     */
    public void enableZeroingMode(boolean enable) {
        winchMotor.enableSoftLimit(SoftLimitDirection.kForward, !enable);
        winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, !enable);
        isZeroing = enable;
    }

    /**
     * Returns whether or not this Climb arm's limit switch at position 0 is pressed.
     */ 
    public boolean isLimitSwitchPressed() {
        return !zeroLimitSwitch.get();
    }

    /**
     * Resets the motor's encoder to 0.
     */
    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}