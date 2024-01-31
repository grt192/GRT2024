package frc.robot.commands.shooter.feed;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;

public class ShooterFeedShootCommand extends Command{
    ShooterFeederSubsystem feederSubsystem;

    public ShooterFeedShootCommand(ShooterFeederSubsystem feederSubsystem){
        this.feederSubsystem = feederSubsystem;
    }
    
    @Override
    public void initialize() {
        feederSubsystem.setFeederMotorSpeed(feederSubsystem.FEEDER_MOTOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        if(feederSubsystem.getProximity() < feederSubsystem.NO_NOTE_TOLERANCE){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setFeederMotorSpeed(0);
    }

}
