package frc.robot.commands.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import static frc.robot.Constants.ShooterConstants;

/** Readies the motors for flywheel. */
public class ShooterFlywheelReadyCommand extends Command {

    ShooterFlywheelSubsystem shooterSubsystem;

    /** Constructor for this command. */
    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterMotorSpeed(ShooterConstants.TOP_SHOOTER_MOTOR_SPEED, ShooterConstants.BOTTOM_SHOOTER_MOTOR_SPEED);
    }

    @Override
    public boolean isFinished() {
        return true; //STUB
    }

    //pivot: vertical, auto-aim
    //feed: load, shoot
    //shooter: stop, ready shooters

    //load note
    //shoot
    //ready shooter
    //vertical 
    //auto-angle

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ready Shooter has been interuppted");
    }
}
