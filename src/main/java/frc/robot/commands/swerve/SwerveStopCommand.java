package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Stops the swerve subsystem. */
public class SwerveStopCommand extends Command {
    SwerveSubsystem swerveSubsystem;

    /** Stops the swerve subsystem.
     *
     * @param swerveSubsystem The swerveSubsystem to stop.
     */
    public SwerveStopCommand(SwerveSubsystem swerveSubsystem) {
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    /** Stops the swerveSubsystem on initialization. */
    public void initialize() {
        swerveSubsystem.setDrivePowers(0, 0, 0);
    }

    /** Finishes immediately. */
    public boolean isFinished() {
        return true;
    }
}
