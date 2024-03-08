package frc.robot.commands.auton;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** SetCalculatedAngleCommand. */
public class SetCalculatedAngleCommand extends Command {
    private final SwerveSubsystem swerve; 
    
    /** Sets the angle of the robot to aim at the speaker. */
    public SetCalculatedAngleCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    } 

    @Override
    public void execute() {
        swerve.setSwerveAimDrivePowers(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getAngleError()) < Units.degreesToRadians(3);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setRobotRelativeDrivePowers(0, 0, 0);
    }

}