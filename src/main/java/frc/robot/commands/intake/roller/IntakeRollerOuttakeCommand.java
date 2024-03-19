package frc.robot.commands.intake.roller;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.util.TrackingTimer;

/** Runs the rollers backwards infinitely. This means you must decorate the command with an end condition. */
public class IntakeRollerOuttakeCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;
    private final TrackingTimer timer;
    private double frontSpeed = -.75;
    private double integrationSpeed = -1;

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
        this.frontSpeed = -speed;
        this.integrationSpeed = -speed;
    }

    public IntakeRollerOuttakeCommand(IntakeRollerSubsystem intakeSubsystem, double frontSpeed, double integrationSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        timer = new TrackingTimer();
        addRequirements(intakeSubsystem);
        this.frontSpeed = -frontSpeed;
        this.integrationSpeed = -integrationSpeed;
    }

    @Override
    public void initialize() {
        timer.reset();
        intakeSubsystem.setRollSpeeds(frontSpeed, integrationSpeed); 
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