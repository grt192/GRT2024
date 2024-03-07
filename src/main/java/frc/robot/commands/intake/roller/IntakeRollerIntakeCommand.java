package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

public class IntakeRollerIntakeCommand extends Command {
    private final IntakeRollersSubsystem intakeSubsystem;
    private final LightBarSubsystem lightBarSubsystem;

    /**
     * intakes note
     * 
     * @param intakeSubsystem intake subsystem
     * @param lightBarSubsystem LED light bar subsystem
     */
    public IntakeRollerIntakeCommand(IntakeRollersSubsystem intakeSubsystem, LightBarSubsystem lightBarSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        lightBarSubsystem.setLightBarStatus(LightBarStatus.INTAKING);
        System.out.println("IntakeRollerIntakeCommand initialized."); // better than the previous keyboard mash
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(.7, .7);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        intakeSubsystem.setAllRollSpeed(0, 0);
        lightBarSubsystem.setLightBarStatus(
                intakeSubsystem.frontSensorNow() ? LightBarStatus.HOLDING_NOTE : LightBarStatus.DORMANT);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.frontSensorNow();
    }
}