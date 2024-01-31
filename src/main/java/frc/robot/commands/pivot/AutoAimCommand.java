package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class AutoAimCommand extends Command{
    
    ShooterPivotSubsystem shooterPivotSubsystem;

    public AutoAimCommand(ShooterPivotSubsystem pivotSubsystem){
        this.shooterPivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    public void initialize(){
        shooterPivotSubsystem.setAutoAimBoolean(true);
    }

    public void end(){
        shooterPivotSubsystem.setAutoAimBoolean(false);
    }

    public boolean isFinished(){
        if(Math.abs(shooterPivotSubsystem.getPosition()) - shooterPivotSubsystem.getCurrentAngle() < shooterPivotSubsystem.ERRORTOLERANCE){
            return true;
        }

        return false;
    }
}
