package frc.robot.commands.climb;

import static frc.robot.Constants.ClimbConstants.RAISE_LIMIT_METERS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

/** Raises both climb arms up to their pre-defined raised position. */
public class ClimbRaiseCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    /** Constructs a {@link ClimbRaiseCommand} for the specified climb subsystem. */
    public ClimbRaiseCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.goToExtension(RAISE_LIMIT_METERS);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtTargetExtension();
    }
}