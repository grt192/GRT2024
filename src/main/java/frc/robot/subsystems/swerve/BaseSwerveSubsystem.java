package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Base of swerve subsystem. Allows easy swapping between swerveSubsystems and test subsystems. */
public class BaseSwerveSubsystem extends SubsystemBase {
    public Pose2d getRobotPosition() {
        return new Pose2d();
    };
}
