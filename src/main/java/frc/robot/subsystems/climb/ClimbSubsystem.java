package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.LEFT_WINCH_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.LEFT_ZERO_LIMIT_PORT;
import static frc.robot.Constants.ClimbConstants.RIGHT_WINCH_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.RIGHT_ZERO_LIMIT_PORT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the climb mechanism (both arms).
 */
public class ClimbSubsystem extends SubsystemBase {
    private final ClimbArm leftClimbArm;
    private final ClimbArm rightClimbArm;

    private static final double ZEROING_SPEED = 0.3;

    private boolean isZeroing;

    /** Constructs a new {@link ClimbSubsystem}. */
    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_PORT, true);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_PORT, false);
    }

    @Override
    public void periodic() {
        leftClimbArm.update();
        rightClimbArm.update();
    }

    /** Sets climb to manual mode (full control over each arm's speed). */
    public void setManual() {
        leftClimbArm.enablePID(false);
        rightClimbArm.enablePID(false);
        leftClimbArm.enableSoftLimits(false);
        rightClimbArm.enableSoftLimits(false);
    }

    /** Sets climb to automatic mode (each arm controlled by a position PID). */
    public void setAutomatic() {
        leftClimbArm.enableSoftLimits(true);
        rightClimbArm.enableSoftLimits(true);
        leftClimbArm.enablePID(true);
        rightClimbArm.enablePID(true);
    }

    /**
     * Sets the targeted extension height for both climb arms.
     *
     * @param height The extension target
     */
    public void goToExtension(double height) {
        if (isZeroing) {
            return;
        }

        leftClimbArm.setExtensionTarget(height);
        rightClimbArm.setExtensionTarget(height);
    }

    /** Returns true if both climb arms are at their extension target, and false otherwise. */
    public boolean isAtTargetExtension() {
        return leftClimbArm.isAtExtensionTarget() && rightClimbArm.isAtExtensionTarget();
    }

    /**
     * Sets the climb arms' speeds from -1.0 (downwards) to +1.0 (upwards)
     *
     * @param leftArmSpeed The left arm's desired speed.
     * @param rightArmSpeed The right arm's desired speed.
     */
    public void setSpeeds(double leftArmSpeed, double rightArmSpeed) {
        if (isZeroing) {
            return;
        }

        leftClimbArm.setSpeed(leftArmSpeed);
        rightClimbArm.setSpeed(rightArmSpeed);
    }

    /** Starts zeroing both climb arms. */
    public void startZeroing() {
        setSpeeds(-ZEROING_SPEED, -ZEROING_SPEED);
        setManual();

        isZeroing = true;
    }
    
    /** Ends the zeroing process for both climb arms. */
    public void endZeroing() {
        isZeroing = false;

        setSpeeds(0, 0);
        goToExtension(0);
        setAutomatic();
    }

    /**
     * Returns whether or not both climb arms are correctly zeroed (and currently at position zero).
     */
    public boolean isZeroedAndAtZero() {
        boolean leftArmZeroedAndAtZero = leftClimbArm.isLimitSwitchPressed() 
                                      && leftClimbArm.getCurrentExtension() == 0;
        boolean rightArmZeroedAndAtZero = rightClimbArm.isLimitSwitchPressed()
                                       && rightClimbArm.getCurrentExtension() == 0;

        return leftArmZeroedAndAtZero && rightArmZeroedAndAtZero;
    }
}