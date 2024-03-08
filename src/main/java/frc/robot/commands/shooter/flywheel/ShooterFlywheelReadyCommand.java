package frc.robot.commands.shooter.flywheel;

import static frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

/** Readies the motors for flywheel. */
public class ShooterFlywheelReadyCommand extends Command {

    ShooterFlywheelSubsystem shooterSubsystem;
    LightBarSubsystem lightBarSubsystem;

    private double topSpeed = ShooterConstants.TOP_SHOOTER_MOTOR_SPEED;
    private double bottomSpeed = ShooterConstants.BOTTOM_SHOOTER_MOTOR_SPEED;

    /** Constructor for this command using default flywheel speed values. */
    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem, LightBarSubsystem lightBarSubsystem) {
        
        this.shooterSubsystem = shooterSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(shooterSubsystem, lightBarSubsystem);

        topSpeed = shooterSubsystem.getTopSpeed();
        bottomSpeed = shooterSubsystem.getBottomSpeed();
    }
    
    /** Constructor for this command. */
    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem, LightBarSubsystem lightBarSubsystem,
            double topSpeed, double bottomSpeed) {
        
        this(shooterSubsystem, lightBarSubsystem);

        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterMotorSpeed(topSpeed, bottomSpeed);
    }

    @Override
    public void execute() {

        double top = shooterSubsystem.getActualTopSpeed() / shooterSubsystem.getTargetTopRPS();
        double bottom = shooterSubsystem.getActualBottomSpeed() / shooterSubsystem.getTargetBottomRPS();
        double avg = (top + bottom) / 2; // in case they're different, this just shows the average. 

        lightBarSubsystem.updateShooterSpeedPercentage(avg);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.atSpeed();
    }

    // pivot: vertical, auto-aim
    // feed: load, shoot
    // shooter: stop, ready shooters

    // load note
    // shoot
    // ready shooter
    // vertical
    // auto-angle

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Ready Shooter has been interrupted");
        }
    }
}