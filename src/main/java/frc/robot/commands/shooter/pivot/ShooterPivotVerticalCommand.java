package frc.robot.commands.shooter.pivot;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

/** Sets the shooter to face directly upwards. */
public class ShooterPivotVerticalCommand extends Command {
    ShooterPivotSubsystem pivotSubsystem;

    /** Constructs a {@link ShooterPivotVerticalCommand} using the specified pivot. */
    public ShooterPivotVerticalCommand(ShooterPivotSubsystem pivotSubsystem) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.setAutoAimBoolean(false);
        pivotSubsystem.setAngle(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pivotSubsystem.getPosition()) < ShooterConstants.PID_ERROR_TOLERANCE);
    }
}