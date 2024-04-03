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
    private static final double KEEP_LOWERED_POWER = -0.2;

    private final ClimbArm leftClimbArm;
    private final ClimbArm rightClimbArm;

    private double leftPower;
    private double rightPower;

    private boolean keepLowered;

    /** Constructs a new {@link ClimbSubsystem}. */
    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_PORT, true);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_PORT, false);

        keepLowered = true;
    }

    @Override
    public void periodic() {
        // System.out.println("Left: " + GRTUtil.twoDecimals(leftClimbArm.getCurrentExtension())
        //               + ", Right: " + GRTUtil.twoDecimals(rightClimbArm.getCurrentExtension()));

        // System.out.println(leftClimbArm.isLimitSwitchPressed() + " " + rightClimbArm.isLimitSwitchPressed());

        leftClimbArm.update((leftPower == 0) && keepLowered ? KEEP_LOWERED_POWER : leftPower);
        rightClimbArm.update((rightPower == 0) && keepLowered ? KEEP_LOWERED_POWER : rightPower);
    }

    /**
     * Sets the climb arms' speeds from -1.0 (downwards) to +1.0 (upwards)
     *
     * @param leftArmSpeed The left arm's desired speed.
     * @param rightArmSpeed The right arm's desired speed.
     */
    public void setSpeeds(double leftArmSpeed, double rightArmSpeed) {
        leftPower = leftArmSpeed;
        rightPower = rightArmSpeed;
    }

    /** When enabled, the climb arms keep themselves at the bottom without manual input. */
    public void enableAutomaticLowering(boolean enable) {
        keepLowered = enable;
    }

    /** Toggles the feature to keep the climb arms at the bottom without manual input. */
    public void toggleAutomaticLowering() {
        keepLowered = !keepLowered;
    }

    /** Returns true if both climb arms are at their lowest positions. */
    public boolean isLowered() {
        return rightClimbArm.isLimitSwitchPressed() && leftClimbArm.isLimitSwitchPressed();
    }
}