package frc.robot.commands.intake.roller;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.IntakeRollerSubsystem;

public class IntakeRollerAmpIntakeCommand extends Command {

    private final IntakeRollerSubsystem intakeRollerSubsystem;

    public IntakeRollerAmpIntakeCommand (IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void initialize() {
       intakeRollerSubsystem.setRollSpeeds(0.3, 0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SENSOR REACHED");
        intakeRollerSubsystem.setRollSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        return intakeRollerSubsystem.getFrontSensorReached();
    }

}