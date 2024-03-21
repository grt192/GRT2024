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
        addRequirements(shooterSubsystem);

        topSpeed = shooterSubsystem.getTopMotorSplineSpeed();
        bottomSpeed = shooterSubsystem.getBottomMotorSplineSpeed();
        shooterSubsystem.setAutoAimShooter(true);
    }
    
    /** Constructor for this command. */
    public ShooterFlywheelReadyCommand(ShooterFlywheelSubsystem shooterSubsystem, LightBarSubsystem lightBarSubsystem,
            double topSpeed, double bottomSpeed) {
        
        this.shooterSubsystem = shooterSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(shooterSubsystem);

        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
        shooterSubsystem.setAutoAimShooter(false);

        System.out.println("Distance to Shooeter: " + shooterSubsystem.returnShootingDistance());
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterMotorSpeed(topSpeed, bottomSpeed);
        
        double top = shooterSubsystem.getTopSpeed() / shooterSubsystem.getTargetTopRPS();
        double bottom = shooterSubsystem.getBottomSpeed() / shooterSubsystem.getTargetBottomRPS();
        double avg = (top + bottom) / 2; // in case they're different, this just shows the average. 

        lightBarSubsystem.updateShooterSpeedPercentage(avg);
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.atSpeed();
    }
}