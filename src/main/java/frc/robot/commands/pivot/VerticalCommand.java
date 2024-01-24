package frc.robot.commands.pivot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class VerticalCommand extends Command{
    PivotSubsystem pivotSubsystem;

    public VerticalCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize() {
        pivotSubsystem.setAngle(90.0);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(pivotSubsystem.getPosition()) - 90 < pivotSubsystem.ERRORTOLERANCE){
            return true;
        }

        return false;
    }
}
