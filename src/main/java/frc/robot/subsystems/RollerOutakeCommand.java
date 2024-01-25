package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TrackingTimer;

public class RollerOutakeCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    public RollerOutakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRollSpeed(1.0,-1.0); 
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
       
        if(intakeSubsystem.sensornow()==false&& timer.hasStarted()==false ){
            timer.start();
        }
            

    }

    @Override
    public void end(boolean interrupted){
        // TODO Auto-generated method stub
    
        intakeSubsystem.setRollSpeed(0,0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
       
    }
}