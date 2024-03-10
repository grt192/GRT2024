package frc.robot.commands.intake.pivot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakePivotSubsystem;





public class IntakePivotMiddleCommand  extends Command{
    private final IntakePivotSubsystem pivotSubsystem;
    private double position;


    /**
     * sets the intake pivot to a middle position
     * @param pivotSubsystem
     */
    public IntakePivotMiddleCommand (IntakePivotSubsystem pivotSubsystem, double position){
        this.pivotSubsystem = pivotSubsystem;
        this.position = position;
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotSubsystem.setPosition(position);
    }
    @Override
    public void execute() {
        System.out.println("PIVOT MOVING" + pivotSubsystem.getEncoderPosition());
        //TODO Auto-generated method stub
    }

    @Override
    public void end(boolean interrupted) {
        //TODO Auto-generated method stub
        System.out.println("PIVOT FINISHED");
        if (interrupted) {
            System.out.println("PIVOT InTERRUPTED");
        }
        pivotSubsystem.movePivot(0);
        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pivotSubsystem.getEncoderPosition() - position) < .1;
        // TODO Auto-generated method stub
       
    }
}