package frc.robot.commands.intake.roller;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.util.TrackingTimer;

/** Runs the rollers forwards infinitely. This means you must decorate the command with an end condition.*/
public class IntakeRollerFeedCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;
    private final TrackingTimer timer;

    /**
     * Sets all the rollers inwards to pass note into shooter.
     *
     * @param intakeRollerSubsystem The {@link IntakeRollerSubsystem} to set the powers to.
     */
    public IntakeRollerFeedCommand(IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeSubsystem = intakeRollerSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRollSpeeds(.6, .6);
        timer.reset();
    }

    @Override
    public void execute() {
        if (intakeSubsystem.getFrontSensorReached() == false && timer.hasStarted() == false) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollSpeeds(0, 0); 
        
    }
}