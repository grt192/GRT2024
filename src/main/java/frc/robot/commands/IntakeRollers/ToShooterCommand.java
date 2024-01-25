package frc.robot.commands.IntakeRollers;

import static frc.robot.Constants.RollerandPivotConstants.pastsensortime;
import static frc.robot.Constants.RollerandPivotConstants.rollersclockwise;
import static frc.robot.Constants.RollerandPivotConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.TrackingTimer;

public class ToShooterCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    public ToShooterCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(rollerscounterclockwise,rollersclockwise);
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
        return timer.hasElapsed(pastsensortime);
    
    }

}