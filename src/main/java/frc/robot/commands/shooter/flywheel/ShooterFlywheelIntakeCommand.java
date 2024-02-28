package frc.robot.commands.shooter.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

/** Intakes from the shooter. */
public class ShooterFlywheelIntakeCommand extends Command {

    ShooterFlywheelSubsystem flywheelSubsystem;

    /** Constructor for the command. */
    public ShooterFlywheelIntakeCommand(ShooterFlywheelSubsystem shooterSubsystem) {
        this.flywheelSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setShooterMotorSpeed(-ShooterConstants.TOP_SHOOTER_MOTOR_SPEED); //turns the other way
    }

    @Override
    public boolean isFinished() {
        return true; //STUB
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ready Shooter has been interuppted");
    }
}
