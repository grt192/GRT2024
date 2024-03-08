package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** DriveForward Command. */
public class DriveForwardCommand extends Command {
    private final SwerveSubsystem swerve;
    private double xPower = .15; 

    /** Sets robot relative powers to drive forward until timed out. */
    public DriveForwardCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setRobotRelativeDrivePowers(xPower, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setRobotRelativeDrivePowers(0, 0, 0);
    }
}