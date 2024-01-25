package frc.robot.commands.IntakePivot;

import static frc.robot.Constants.RollerandPivotConstants.pivotcounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem; 

public class PivotExtendedCommand extends Command{
    private final PivotSubsystem pivotSubsystem;

    public PivotExtendedCommand(PivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotSubsystem.movePivot(pivotcounterclockwise);
    }
    
    @Override
    public void execute() {
        // TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        pivotSubsystem.movePivot(0);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pivotSubsystem.pivotisextended();
    }
}