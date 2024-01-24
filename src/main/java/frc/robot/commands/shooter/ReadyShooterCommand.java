package frc.robot.commands.shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ReadyShooterCommand extends Command{

    ShooterSubsystem shooterSubsystem;

    public ReadyShooterCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterMotorSpeed(shooterSubsystem.SHOOTER_MOTOR_SPEED);
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
