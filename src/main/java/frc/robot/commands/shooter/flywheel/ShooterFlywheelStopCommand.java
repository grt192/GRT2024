package frc.robot.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

/** Stops the shooter flywheels. */
public class ShooterFlywheelStopCommand extends Command {
    ShooterFlywheelSubsystem shooterSubsystem;

    /** Constructs a {@link ShooterFlywheelStopCommand} for the specified shooter. */
    public ShooterFlywheelStopCommand(ShooterFlywheelSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {   
        shooterSubsystem.setShooterMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}