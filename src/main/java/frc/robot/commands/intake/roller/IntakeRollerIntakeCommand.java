package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

/** Runs the rollers on the intake until a note is detected. */
public class IntakeRollerIntakeCommand extends Command {
    private final IntakeRollerSubsystem intakeSubsystem;
    private final LightBarSubsystem lightBarSubsystem;
    private double frontSpeed = .7;
    private double integrationSpeed = .4;

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

    public IntakeRollerIntakeCommand(IntakeRollerSubsystem intakeSubsystem, LightBarSubsystem lightBarSubsystem, double speed, double integrationSpeed) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeSubsystem);
        this.frontSpeed = speed;
        this.integrationSpeed = integrationSpeed;
    }

    @Override
    public void initialize() {
        lightBarSubsystem.setLightBarStatus(LightBarStatus.INTAKING, 2);
    }

    @Override
    public void execute() {
        intakeSubsystem.setRollSpeeds(frontSpeed, integrationSpeed);
        System.out.println(intakeSubsystem.getRockwellSensorValue());
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