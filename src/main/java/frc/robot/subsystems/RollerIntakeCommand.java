package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;

public class RollerIntakeCommand extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public RollerIntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        intakeSubsystem.setRollSpeed(-1.0,1.0); 
    }

    @Override
    public void end(boolean interrupted){
        // TODO Auto-generated method stub
        intakeSubsystem.setRollSpeed(0,0); 
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.sensornow();
    }
}