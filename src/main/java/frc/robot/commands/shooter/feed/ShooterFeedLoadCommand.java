package frc.robot.commands.shooter.feed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;

public class ShooterFeedLoadCommand extends Command {
    
    ShooterFeederSubsystem feederSubsystem;

    public ShooterFeedLoadCommand(ShooterFeederSubsystem feederSubsystem){
        this.feederSubsystem = feederSubsystem;
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.setFeederMotorSpeed(feederSubsystem.FEEDER_MOTOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        if(feederSubsystem.getProximity() > feederSubsystem.TOLERANCE){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setFeederMotorSpeed(0);
    }
}
