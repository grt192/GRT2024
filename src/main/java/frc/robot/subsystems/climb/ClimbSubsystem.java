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

    /** Constructs a new {@link ClimbSubsystem}. */
    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_PORT, true);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_PORT, false);
    }

    @Override
    public void periodic() {
        // System.out.println("Left: " + GRTUtil.twoDecimals(leftClimbArm.getCurrentExtension())
        //               + ", Right: " + GRTUtil.twoDecimals(rightClimbArm.getCurrentExtension()));

        // System.out.println(leftClimbArm.isLimitSwitchPressed() + " " + rightClimbArm.isLimitSwitchPressed());

        leftClimbArm.update();
        rightClimbArm.update();
    }

    /**
     * Sets the climb arms' speeds from -1.0 (downwards) to +1.0 (upwards)
     *
     * @param leftArmSpeed The left arm's desired speed.
     * @param rightArmSpeed The right arm's desired speed.
     */
    public void setSpeeds(double leftArmSpeed, double rightArmSpeed) {
        leftClimbArm.setSpeed(leftArmSpeed);
        rightClimbArm.setSpeed(rightArmSpeed);
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