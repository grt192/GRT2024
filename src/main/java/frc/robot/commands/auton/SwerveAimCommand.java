package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** SetCalculatedAngleCommand. */
public class SwerveAimCommand extends Command {
    private final SwerveSubsystem swerve; 
    
    /** Sets the angle of the robot to aim at the speaker. */
    public SwerveAimCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
    } 

    @Override
    public void initialize() {
        swerve.targetSpeaker();
        swerve.setAim(true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setAim(false);
    }
}