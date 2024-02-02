package frc.robot.commands.shooter.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterPivotSetAngle extends Command{
    ShooterPivotSubsystem pivotSubsystem;
    double angle;

    public ShooterPivotSetAngle(ShooterPivotSubsystem pivotSubsystem, double angle){
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        pivotSubsystem.setAutoAimBoolean(false);
        pivotSubsystem.setAngle(angle);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(pivotSubsystem.getPosition()) - angle < pivotSubsystem.ERRORTOLERANCE){
            return true;
        }

        return false;
    }
}