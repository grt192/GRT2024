package frc.robot.commands.shooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class StopShooterCommands extends Command{
    ShooterSubsystem shooterSubsystem;

    public StopShooterCommands(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){   
        shooterSubsystem.setShooterMotorSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}


