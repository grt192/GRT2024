package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

/** Runs the rollers on the intake until a note is detected. */
public class IntakeRollerIntakeCommand extends Command {
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final LightBarSubsystem lightBarSubsystem;

    private boolean colorSensorDead;
    private Timer timer = new Timer();
    private double speed = .7;

    /**
     * Runs the rollers on the intake until a note is detected.
     *
     * @param intakeRollerSubsystem The {@link IntakeRollerSubsystem} to run the rollers on
     * @param lightBarSubsystem The {@link LightBarSubsystem} to display the intaking on
     */
    public IntakeRollerIntakeCommand(IntakeRollerSubsystem intakeRollerSubsystem, LightBarSubsystem lightBarSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeRollerSubsystem);

        colorSensorDead = intakeRollerSubsystem.getColorSensorRed() == 0;
        if (colorSensorDead) System.out.println("Color sensor dead, falling back on distance sensor.");
    }

    /**
     * Runs the rollers on the intake until a note is detected.
     *
     * @param intakeRollerSubsystem The {@link IntakeRollerSubsystem} to run the rollers on
     * @param lightBarSubsystem The {@link LightBarSubsystem} to display the intaking on
     * @param speed The speed to run the rollers at
     */
    public IntakeRollerIntakeCommand(
        IntakeRollerSubsystem intakeRollerSubsystem, LightBarSubsystem lightBarSubsystem, double speed
    ) {
        this(intakeRollerSubsystem, lightBarSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        lightBarSubsystem.setLightBarStatus(LightBarStatus.INTAKING);
    }

    @Override
    public void execute() {
        intakeRollerSubsystem.setRollSpeeds(speed, speed);
        if (intakeRollerSubsystem.getBackSensorReached()) {
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeRollerSubsystem.setRollSpeeds(0, 0);
        if (!interrupted) lightBarSubsystem.setLightBarStatus(LightBarStatus.HOLDING_NOTE);
    }

    @Override
    public boolean isFinished() {
        if (colorSensorDead) {
            return timer.hasElapsed(IntakeConstants.INTAKE_FEED_TIME);
        } else {
            if (intakeRollerSubsystem.getNoteColorDetected()) {
                System.out.println("Time to color sensor: " + timer.get());
                return true;
            }
            return false;
        }
        
    }
}