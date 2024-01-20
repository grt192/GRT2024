package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private final ClimbArm leftClimbArm;
    private final ClimbArm rightClimbArm;

    public ClimbSubsystem() {
        leftClimbArm = new ClimbArm(LEFT_WINCH_MOTOR_ID, LEFT_ZERO_LIMIT_ID);
        rightClimbArm = new ClimbArm(RIGHT_WINCH_MOTOR_ID, RIGHT_ZERO_LIMIT_ID);
    }

    @Override
    public void periodic() {
        leftClimbArm.update();
        rightClimbArm.update();
    }

    public void goToExtension(double height) {
        leftClimbArm.goToExtension(height);
        rightClimbArm.goToExtension(height);
    }

    public boolean isAtExtension() {
        return leftClimbArm.isAtExtension() && rightClimbArm.isAtExtension();
    }
}
