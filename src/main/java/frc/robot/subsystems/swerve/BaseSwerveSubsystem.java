package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Base of swerve subsystem. Allows easy swapping between swerveSubsystems and test subsystems. */
public class BaseSwerveSubsystem extends SubsystemBase {

    /**
     * Gets the current robot position.
     *
     * @return The current x, y, and theta of the robot in the form of a {@link Pose2d}
     */
    public Pose2d getRobotPosition() {
        return new Pose2d();
    }
}
