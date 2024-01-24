package frc.robot.commands.feeder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class ShootNoteCommand extends Command{
    FeederSubsystem feederSubsystem;

    public ShootNoteCommand(FeederSubsystem feederSubsystem){
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
