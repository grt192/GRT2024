package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveStopCommand extends Command {
    SwerveSubsystem swerveSubsystem;

    public SwerveStopCommand(SwerveSubsystem swerveSubsystem){
        addRequirements(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    public void initialize() {
        swerveSubsystem.setDrivePowers(0, 0, 0);
    }
    
    public boolean isFinished() {
        return true;
    }
}
