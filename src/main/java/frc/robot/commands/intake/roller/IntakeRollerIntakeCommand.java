package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

/** Runs the rollers on the intake until a note is detected. */
public class IntakeRollerIntakeCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;
    private final LightBarSubsystem lightBarSubsystem;
    private double speed = .7;

    /**
     * Runs the rollers on the intake until a note is detected.
     *
     * @param intakeSubsystem The {@link IntakeRollerSubsystem} to run the rollers on
     * @param lightBarSubsystem The {@link LightBarSubsystem} to display the intaking on
     */
    public IntakeRollerIntakeCommand(IntakeRollerSubsystem intakeSubsystem, LightBarSubsystem lightBarSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeSubsystem);
    }

    public IntakeRollerIntakeCommand(IntakeRollerSubsystem intakeSubsystem, LightBarSubsystem lightBarSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        lightBarSubsystem.setLightBarStatus(LightBarStatus.INTAKING, 2);
    }

    @Override
    public void execute() {
        intakeSubsystem.setRollSpeeds(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setRollSpeeds(0, 0);
        lightBarSubsystem.setLightBarStatus(LightBarStatus.HOLDING_NOTE, 2);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getRockwellSensorValue();
    }
}