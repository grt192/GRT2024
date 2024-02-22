package frc.robot.commands.intake.roller;


import static frc.robot.Constants.IntakeConstants.pastsensortime;
import static frc.robot.Constants.IntakeConstants.rollersclockwise;
import static frc.robot.Constants.IntakeConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerOutakeCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    /**
     * sets all the rollers outwards to outake
     * @param intakeSubsystem
     */
    public IntakeRollerOutakeCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.setAllRollSpeed(-1,-1); 
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
       
        if(intakeSubsystem.sensorNow()==false&& timer.hasStarted()==false ){
            timer.start();
        }

    }

    @Override
    public void end(boolean interrupted){
        // TODO Auto-generated method stub
    
        intakeSubsystem.setAllRollSpeed(0,0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(pastsensortime);
       
    }
}