package frc.robot.commands.intake.roller;

import static frc.robot.Constants.IntakeConstants.pastsensortime;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerFeedCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    /**
     * Sets all the rollers inwards to pass note into shooter
     * @param intakeSubsystem
     */
    public IntakeRollerFeedCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(.6, .6);
        timer.reset();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (intakeSubsystem.frontSensorNow() == false && timer.hasStarted() == false) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(0, 0); 
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.hasElapsed(pastsensortime);
    
    }

}