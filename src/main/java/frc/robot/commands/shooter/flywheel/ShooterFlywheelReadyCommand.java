package frc.robot.commands.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import static frc.robot.Constants.ShooterConstants;

/** Readies the motors for flywheel. */
public class ShooterFlywheelReadyCommand extends Command {

    ShooterFlywheelSubsystem shooterSubsystem;

    private double topSpeed = ShooterConstants.TOP_SHOOTER_MOTOR_SPEED;
    private double bottomSpeed = ShooterConstants.BOTTOM_SHOOTER_MOTOR_SPEED;

    /** Constructor for this command. */
    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        topSpeed = shooterSubsystem.getTopSpeed();
        bottomSpeed = shooterSubsystem.getBottomSpeed();
    }

    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem, double topSpeed, double bottomSpeed) {
        this(shooterSubsystem);
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterMotorSpeed(topSpeed, bottomSpeed);
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
