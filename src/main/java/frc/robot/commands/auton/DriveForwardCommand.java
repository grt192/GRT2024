package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveForwardCommand extends Command{
    private final SwerveSubsystem swerve;

    public DriveForwardCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }
}
