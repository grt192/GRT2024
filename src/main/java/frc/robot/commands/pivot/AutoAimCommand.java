package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class AutoAimCommand extends Command{
    
    PivotSubsystem pivotSubsystem;

    public AutoAimCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    public void initialize(){
        pivotSubsystem.setAutoAimBoolean(true);
    }

    public void end(){
        pivotSubsystem.setAutoAimBoolean(false);
    }

    public boolean isFinished(){
        if(Math.abs(pivotSubsystem.getPosition()) - pivotSubsystem.getCurrentAngle() < pivotSubsystem.ERRORTOLERANCE){
            return true;
        }

        return false;
    }
}
