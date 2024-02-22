
package frc.robot.commands.intake.pivot;


import static frc.robot.Constants.IntakeConstants.PIVOT_CLOCKWISE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;

public class IntakePivotVerticalCommand extends Command{
    private final IntakePivotSubsystem pivotSubsystem;

    public IntakePivotVerticalCommand(IntakePivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotSubsystem.movePivot(PIVOT_CLOCKWISE);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
      
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        pivotSubsystem.movePivot(0);
        //pivotSubsystem.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pivotSubsystem.pivotisextended();
    
    }

}