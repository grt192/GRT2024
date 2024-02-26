package frc.robot.commands.intake.roller;

import static frc.robot.Constants.IntakeConstants.rollerscounterclockwise;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.NotePosition;

public class IntakeRollerIntakeCommand extends Command{
    private final IntakeRollersSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    /**
     * intakes note
     * @param intakeSubsystem
     * @param ledSubsystem
     */
    public IntakeRollerIntakeCommand(IntakeRollersSubsystem intakeSubsystem, LEDSubsystem ledSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.setNoteMode(NotePosition.INTAKING);
        System.out.println("SLDKJFHDSFLJKHDSFKHJDSFLKJKJLD");
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
        ledSubsystem.setNoteMode(intakeSubsystem.sensorNow() ? NotePosition.INTAKE_HOLDING : NotePosition.NONE);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.sensorNow();
    }
}