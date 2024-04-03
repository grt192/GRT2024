package frc.robot.commands.shooter.pivot;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

/** Aims the shooter at its target. */
public class ShooterPivotAimCommand extends Command {
    
    ShooterPivotSubsystem shooterPivotSubsystem;

    /** Constructs a {@link ShooterPivotAimCommand} using the specified pivot. Sets auto-aim to true. */
    public ShooterPivotAimCommand(ShooterPivotSubsystem pivotSubsystem) {
        this.shooterPivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        shooterPivotSubsystem.setAutoAimBoolean(true);
    }

    @Override
    public void end(boolean interrupted) {
        shooterPivotSubsystem.setAutoAimBoolean(false);
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(shooterPivotSubsystem.getPosition() - shooterPivotSubsystem.getAutoAimAngle())
    //         < ShooterConstants.PID_ERROR_TOLERANCE;
    // }
}