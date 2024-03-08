package frc.robot.commands.shooter.pivot;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

/** Sets the shooter to aim at a given angle. */
public class ShooterPivotSetAngleCommand extends Command {
    ShooterPivotSubsystem pivotSubsystem;
    double angle;

    /**
     * Constructs a {@link ShooterPivotSetAngleCommand} using the specified pivot and angle.
     *
     * @param angle The target angle in degrees below the vertical.
     */
    public ShooterPivotSetAngleCommand(ShooterPivotSubsystem pivotSubsystem, double angle) {
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        System.out.println("SET ANGLE");
        pivotSubsystem.setAutoAimBoolean(false);
        pivotSubsystem.setAngle(angle);
    }

    @Override
    public boolean isFinished() {
        System.out.println("NOT FINISHED" + Math.abs(pivotSubsystem.getPosition() - angle));
        return (Math.abs(pivotSubsystem.getPosition() - angle) < ShooterConstants.PID_ERROR_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("ANGLE INTERRUPTED");
        } else {
            System.out.println("ANGLE ARRIVED");
        }
    }
}