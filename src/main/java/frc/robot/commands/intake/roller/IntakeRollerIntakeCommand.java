package frc.robot.commands.intake.roller;


import static frc.robot.Constants.RollerandPivotConstants.rollersclockwise;
import static frc.robot.Constants.RollerandPivotConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;

public class IntakeRollerIntakeCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;

    public IntakeRollerIntakeCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        intakeSubsystem.setRollSpeed(rollerscounterclockwise,rollersclockwise); 
    }

    @Override
    public void end(boolean interrupted){
        // TODO Auto-generated method stub
        intakeSubsystem.setRollSpeed(0,0); 
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.sensorNow();
    }
}