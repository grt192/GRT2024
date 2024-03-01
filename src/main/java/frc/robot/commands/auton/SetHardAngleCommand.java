package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetHardAngleCommand extends Command{
    private final SwerveSubsystem swerve; 
    private final Rotation2d targetAngle;
   
    public SetHardAngleCommand(SwerveSubsystem swerve, Rotation2d targetAngle){
        this.swerve = swerve;
        this.targetAngle = targetAngle;
        addRequirements(swerve);
    } 

    @Override
    public void execute() {
        swerve.setDrivePowersWithHeadingLock(0,0, targetAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getRobotPosition().getRotation().minus(targetAngle).getDegrees()) < 2;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setRobotRelativeDrivePowers(0,0,0);
    }

}