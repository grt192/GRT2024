package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Climb mech (both arms)
 */
public class ClimbSubsystem extends SubsystemBase{
    private final ClimbArm leftClimbArm;
    private final ClimbArm rightClimbArm;

    /**
     * Constructs a new {@link ClimbSubsystem}
     */
    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_ID);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_ID);
    }

    @Override
    public void periodic() {
        leftClimbArm.update();
        rightClimbArm.update();
    }

    /**
     * Sets the targeted extension height for both climb arms
     * @param height The extension target
     */
    public void goToExtension(double height) {
        leftClimbArm.setTargetExtension(height);
        rightClimbArm.setTargetExtension(height);
    }

    /**
     * @return True if both climb arms are at their extension target, False otherwise
     */
    public boolean isAtTargetExtension() {
        return leftClimbArm.isAtTargetExtension() && rightClimbArm.isAtTargetExtension();
    }
}
