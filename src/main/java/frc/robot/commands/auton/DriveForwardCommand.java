package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveForwardCommand extends Command{
    private final SwerveSubsystem swerve;
    private int xpower;
    private int ypower;

    public DriveForwardCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setDrivePowers(xpower, ypower, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDrivePowers(0,0,0);
    }

}
