package frc.robot.commands.shooter.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

public class ShooterFlywheelStopCommand extends Command{
    ShooterFlywheelSubsystem shooterSubsystem;

    public ShooterFlywheelStopCommand(ShooterFlywheelSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {   
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}



