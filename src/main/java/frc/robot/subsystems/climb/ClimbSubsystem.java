package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.LEFT_SOLENOID_LATCH_PORT;
import static frc.robot.Constants.ClimbConstants.LEFT_WINCH_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.LEFT_ZERO_LIMIT_PORT;
import static frc.robot.Constants.ClimbConstants.RIGHT_SOLENOID_LATCH_PORT;
import static frc.robot.Constants.ClimbConstants.RIGHT_WINCH_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.RIGHT_ZERO_LIMIT_PORT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Climb mechanism (both arms).
 */
public class ClimbSubsystem extends SubsystemBase {
    private final ClimbArm leftClimbArm;
    private final ClimbArm rightClimbArm;

    /**
     * Constructs a new {@link ClimbSubsystem}.
     */
    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_PORT, LEFT_SOLENOID_LATCH_PORT);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_PORT, RIGHT_SOLENOID_LATCH_PORT);

    }

    @Override
    public void periodic() {
        leftClimbArm.update();
        rightClimbArm.update();

        /* The latches consume ~50 W combined when open, so they should be closed when the arms are not moving. */
        if (this.isAtTargetExtension()) {
            closeLatches();
        }
    }

    /**
     * Sets the targeted extension height for both Climb arms.
     *
     * @param height The extension target
     */
    public void goToExtension(double height) {
        /* The latches must be open for the climb hooks to move past them. */
        openLatches();

        leftClimbArm.setTargetExtension(height);
        rightClimbArm.setTargetExtension(height);
    }

    /**
     * Returns true if both Climb arms are at their extension target, and false otherwise.
     */
    public boolean isAtTargetExtension() {
        return leftClimbArm.isAtTargetExtension() && rightClimbArm.isAtTargetExtension();
    }

    /**
     * Starts zeroing both Climb arms.
     */
    public void startZeroing() {
        leftClimbArm.enableZeroingMode(true);
        rightClimbArm.enableZeroingMode(true);
    }
    
    /**
     * Ends the zeroing process for both climb arms.
     */
    public void endZeroing() {
        leftClimbArm.enableZeroingMode(false);
        rightClimbArm.enableZeroingMode(false);
    }

    /**
     * Returns whether or not both climb arms are correctly zeroed (and currently at position zero).
     */
    public boolean isZeroedAndAtZero() {
        boolean leftArmZeroedAndAtZero = leftClimbArm.isLimitSwitchPressed() && leftClimbArm.getCurrentExtension() == 0;
        boolean rightArmZeroedAndAtZero = rightClimbArm.isLimitSwitchPressed() && rightClimbArm.getCurrentExtension() == 0;

        return leftArmZeroedAndAtZero && rightArmZeroedAndAtZero;
    }

    private void closeLatches() {
        leftClimbArm.closeLatch();
        rightClimbArm.closeLatch();
    }

    private void openLatches() {
        leftClimbArm.openLatch();
        rightClimbArm.openLatch();
    }
}