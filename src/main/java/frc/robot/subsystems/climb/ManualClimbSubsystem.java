package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.LEFT_WINCH_MOTOR_ID;
import static frc.robot.Constants.ClimbConstants.RIGHT_WINCH_MOTOR_ID;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Climb mechanism (both arms).
 */
public class ManualClimbSubsystem extends SubsystemBase {
    private final ManualClimbArm leftClimbArm;
    private final ManualClimbArm rightClimbArm;

    /**
     * Constructs a new {@link ClimbSubsystem}.
     */
    public ManualClimbSubsystem() {
        leftClimbArm = new ManualClimbArm(LEFT_WINCH_MOTOR_ID, false);
        rightClimbArm = new ManualClimbArm(RIGHT_WINCH_MOTOR_ID, true);

    }

    @Override
    public void periodic() {
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
}