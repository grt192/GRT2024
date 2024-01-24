
package frc.robot.commands.IntakePivot;


import static frc.robot.Constants.RollerandPivotConstants.pivotclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivotSubsystem;

public class PivotVerticalCommand extends Command{
    private final IntakePivotSubsystem pivotSubsystem;

    public PivotVerticalCommand(IntakePivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotSubsystem.movePivot(pivotclockwise);
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