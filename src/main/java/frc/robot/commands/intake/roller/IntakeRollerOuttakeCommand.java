package frc.robot.commands.intake.roller;


import static frc.robot.Constants.IntakeConstants.pastsensortime;
import static frc.robot.Constants.IntakeConstants.rollersclockwise;
import static frc.robot.Constants.IntakeConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerOuttakeCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    private double speed = -1;
    private double integrationSpeed = -1;

    public IntakeRollerOuttakeCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    public IntakeRollerOuttakeCommand(IntakeRollersSubsystem intakeSubsystem, double speed) {
        this(intakeSubsystem);
        this.speed = -speed;
        integrationSpeed = -speed;
    }

    public IntakeRollerOuttakeCommand(IntakeRollersSubsystem intakeSubsystem, double mainSpeed, double integrationSpeed){
        this(intakeSubsystem);
        this.speed = -mainSpeed;
        this.integrationSpeed = -integrationSpeed;

    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.setAllRollSpeed(speed, integrationSpeed); 
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
       
        if(intakeSubsystem.frontSensorNow()==false&& timer.hasStarted()==false ){
            timer.start();
        }

    }

    @Override
    public void end(boolean interrupted){
        // TODO Auto-generated method stub
    
        intakeSubsystem.setAllRollSpeed(0,0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(pastsensortime);
       
    }
}