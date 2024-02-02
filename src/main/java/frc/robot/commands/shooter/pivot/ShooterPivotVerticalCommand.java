package frc.robot.commands.shooter.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;

public class ShooterPivotVerticalCommand extends Command{
    ShooterPivotSubsystem pivotSubsystem;

    public ShooterPivotVerticalCommand(ShooterPivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        pivotSubsystem.setAutoAimBoolean(false);
        pivotSubsystem.setAngle(0.0);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pivotSubsystem.getPosition()) < pivotSubsystem.ERRORTOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            System.out.println("VERTICAL INTERRUPTED");
        } else {
            System.out.println("VERTICAL ARRIVED");
        }
    }
}