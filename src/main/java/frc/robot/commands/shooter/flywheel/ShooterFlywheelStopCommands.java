package frc.robot.commands.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

public class ShooterFlywheelStopCommands extends Command{
    ShooterFlywheelSubsystem shooterSubsystem;

    public ShooterFlywheelStopCommands(ShooterFlywheelSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
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



