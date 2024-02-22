package frc.robot.commands.intake.pivot;

import static frc.robot.Constants.IntakeConstants.encodermiddle;
import static frc.robot.Constants.IntakeConstants.pivotcounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;





public class IntakePivotMiddleCommand  extends Command{
    private final IntakePivotSubsystem pivotSubsystem;


    /**
     * sets the intake pivot to a middle position
     * @param pivotSubsystem
     */
    public IntakePivotMiddleCommand (IntakePivotSubsystem pivotSubsystem){
        this.pivotSubsystem = pivotSubsystem;
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotSubsystem.setPosition(encodermiddle);
    }
    @Override
    public void execute() {
        //TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        //TODO Auto-generated method stub
        pivotSubsystem.movePivot(0);
        
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.encoderPosition() == encodermiddle;
        // TODO Auto-generated method stub
       
    }
}