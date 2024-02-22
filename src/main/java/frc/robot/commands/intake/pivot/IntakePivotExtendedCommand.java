package frc.robot.commands.intake.pivot;

import static frc.robot.Constants.IntakeConstants.pivotcounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem; 

public class IntakePivotExtendedCommand extends Command{
    private final IntakePivotSubsystem pivotSubsystem;

    /**
     * extends intake 
     * @param pivotSubsystem
     */
    public IntakePivotExtendedCommand(IntakePivotSubsystem pivotSubsystem){
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