package frc.robot.commands.intake.roller;

import static frc.robot.Constants.IntakeConstants.pastsensortime;
import static frc.robot.Constants.IntakeConstants.rollersclockwise;
import static frc.robot.Constants.IntakeConstants.rollerscounterclockwise;
import static frc.robot.Constants.IntakeConstants.pastsensortime;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.util.TrackingTimer;

public class IntakeRollerFeedCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    private double speed = .6;

    /**
     * Sets all the rollers inwards to pass note into shooter
     * @param intakeSubsystem
     */
    public IntakeRollerFeedCommand(IntakeRollersSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    public IntakeRollerFeedCommand(IntakeRollersSubsystem intakeSubsystem, double speed){
        this(intakeSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(speed, speed);
        timer.reset();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (intakeSubsystem.frontSensorNow() == false && timer.hasStarted() == false) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(0, 0); 
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.hasElapsed(pastsensortime);
    
    }

}