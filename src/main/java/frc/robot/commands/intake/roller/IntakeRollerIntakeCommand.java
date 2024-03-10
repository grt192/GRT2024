package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

/** Runs the rollers on the intake until a note is detected. */
public class IntakeRollerIntakeCommand extends Command {
    private final IntakeRollersSubsystem intakeSubsystem;
    private final LightBarSubsystem lightBarSubsystem;

    /**
     * Runs the rollers on the intake until a note is detected.
     *
     * @param intakeSubsystem The {@link IntakeRollersSubsystem} to run the rollers on
     * @param lightBarSubsystem The {@link LightBarSubsystem} to display the intaking on
     */
    public IntakeRollerIntakeCommand(IntakeRollersSubsystem intakeSubsystem, LightBarSubsystem lightBarSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        lightBarSubsystem.setLightBarStatus(LightBarStatus.INTAKING);
        // System.out.println("IntakeRollerIntakeCommand initialized."); 
    }

    @Override
    public void execute() {
        intakeSubsystem.setAllRollSpeed(.7, .7);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setAllRollSpeed(0, 0);
        lightBarSubsystem.setLightBarStatus(
                intakeSubsystem.frontSensorNow() ? LightBarStatus.HOLDING_NOTE : LightBarStatus.DORMANT);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getNoteColorDetected();
    }
}