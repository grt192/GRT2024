package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetCalculatedAngleCommand extends Command{
    private final SwerveSubsystem swerve; 
    

   
    public SetCalculatedAngleCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    } 

    @Override
    public void initialize() {
        swerve.setSwerveAimDrivePowers(0,0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getRobotPosition().getRotation().minus(new Rotation2d(swerve.getShootAngle(false))).getDegrees()) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setRobotRelativeDrivePowers(0,0,0);
    }

}