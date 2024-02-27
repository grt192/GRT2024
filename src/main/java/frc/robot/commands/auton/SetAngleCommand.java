package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SetAngleCommand extends Command{
    private final SwerveSubsystem swerve; 
    

   
    public SetAngleCommand(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    } 

    @Override
    public void initialize() {
        swerve.setAimMode(0,0);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setRobotRelativeDrivePowers(0,0,0);
    }

}