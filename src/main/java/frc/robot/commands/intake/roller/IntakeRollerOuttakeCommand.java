package frc.robot.commands.intake.roller;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.util.TrackingTimer;

/** Runs the rollers backwards infinitely. This means you must decorate the command with an end condition. */
public class IntakeRollerOuttakeCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;
    private final TrackingTimer timer;
    private double speed = -1;

    /**
     * Runs the rollers outwards.
     *
     * @param intakeSubsystem The {@link IntakeRollerSubsystem} to outtake on.
     */
    public IntakeRollerOuttakeCommand(IntakeRollerSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
    }

    /**
     * Runs the rollers outwards at a set speed.
     *
     * @param intakeSubsystem The {@link IntakeRollerSubsystem} to outtake on.
     * @param speed The speed to outtake at [0,1].
     */
    public IntakeRollerOuttakeCommand(IntakeRollerSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
        this.speed = -speed;
    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.setRollSpeeds(speed, speed); 
    }

    @Override
    public void execute() {
        if (intakeSubsystem.getFrontSensorReached() && !timer.hasStarted()) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollSpeeds(0, 0);
    }
}