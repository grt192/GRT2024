package frc.robot.commands.shooter.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterPivotSetAngleCommand extends Command{
    ShooterPivotSubsystem pivotSubsystem;
    double angle;

    public ShooterPivotSetAngleCommand(ShooterPivotSubsystem pivotSubsystem, double angle){
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
        return (Math.abs(pivotSubsystem.getPosition() - angle) < pivotSubsystem.ERRORTOLERANCE);
    }

    public void end(boolean interrupted) {
        if(interrupted){
            System.out.println("ANGLE INTERRUPTED");
        } else {
            System.out.println("ANGLE ARRIVED");
        }
    }
}