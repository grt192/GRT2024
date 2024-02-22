package frc.robot.commands.intake.roller;

import static frc.robot.Constants.IntakeConstants.PAST_SENSOR_TIME;
import static frc.robot.Constants.IntakeConstants.ROLLERS_CLOCKWISE;
import static frc.robot.Constants.IntakeConstants.ROLLERS_COUNTERCLOCKWISE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerFeedCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    public IntakeRollerFeedCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(ROLLERS_COUNTERCLOCKWISE, ROLLERS_CLOCKWISE);
        timer.reset();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if(intakeSubsystem.sensorNow() == false && timer.hasStarted()==false){
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(0,0);
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.hasElapsed(PAST_SENSOR_TIME);
    
    }

}