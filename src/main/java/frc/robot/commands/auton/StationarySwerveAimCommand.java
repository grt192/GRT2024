package frc.robot.commands.auton;

import frc.robot.subsystems.swerve.SwerveSubsystem;

public class StationarySwerveAimCommand extends SwerveAimCommand{

    SwerveSubsystem swerveSubsystem;

    public StationarySwerveAimCommand(SwerveSubsystem swerveSubsystem){
        super(swerveSubsystem);
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
        swerveSubsystem.setChassisSpeeds(0, 0, 0);
    }
}
