package frc.robot.commands.IntakeRollers;


import static frc.robot.Constants.RollerandPivotConstants.pastsensortime;
import static frc.robot.Constants.RollerandPivotConstants.rollersclockwise;
import static frc.robot.Constants.RollerandPivotConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
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
        intakeSubsystem.setRollSpeed(rollersclockwise,rollerscounterclockwise); 
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
    
        intakeSubsystem.setRollSpeed(0,0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(pastsensortime);
       
    }
}