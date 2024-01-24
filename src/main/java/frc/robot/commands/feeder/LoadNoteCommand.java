package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;

public class LoadNoteCommand extends Command {
    
    FeederSubsystem feederSubsystem;

    public LoadNoteCommand(FeederSubsystem feederSubsystem){
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
