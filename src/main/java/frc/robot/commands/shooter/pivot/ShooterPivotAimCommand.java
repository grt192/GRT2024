package frc.robot.commands.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterPivotAimCommand extends Command{
    
    ShooterPivotSubsystem shooterPivotSubsystem;

    public ShooterPivotAimCommand(ShooterPivotSubsystem pivotSubsystem){
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
