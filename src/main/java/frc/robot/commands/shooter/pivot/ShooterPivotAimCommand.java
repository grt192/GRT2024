package frc.robot.commands.shooter.pivot;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterPivotAimCommand extends Command {
    
    ShooterPivotSubsystem shooterPivotSubsystem;

    public ShooterPivotAimCommand(ShooterPivotSubsystem pivotSubsystem){
        this.shooterPivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    public void initialize() {
        shooterPivotSubsystem.setAutoAimBoolean(true);
    }

    public void end() {
        shooterPivotSubsystem.setAutoAimBoolean(false);
    }

    public boolean isFinished() {
        return(Math.abs(shooterPivotSubsystem.getPosition() - shooterPivotSubsystem.getAutoAimAngle()) < ShooterConstants.PID_ERROR_TOLERANCE);

    }
}
