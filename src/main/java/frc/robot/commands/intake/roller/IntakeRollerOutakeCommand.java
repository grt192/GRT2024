package frc.robot.commands.intake.roller;


import static frc.robot.Constants.RollerandPivotConstants.pastsensortime;
import static frc.robot.Constants.RollerandPivotConstants.rollersclockwise;
import static frc.robot.Constants.RollerandPivotConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerOutakeCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    public IntakeRollerOutakeCommand(IntakeRollersSubsystem intakeSubsystem){
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