package frc.robot.commands.climb;

import static frc.robot.Constants.ClimbConstants.LOWER_LIMIT_METERS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

/** Lowers both climb arms down to their pre-defined lowered position. */
public class ClimbLowerCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    /** Constructs a {@link ClimbLowerCommand} for the specified climb subsystem. */
    public ClimbLowerCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("LOWERING CLIMB...");
        climbSubsystem.goToExtension(LOWER_LIMIT_METERS);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtTargetExtension();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted ? "CLIMB LOWERING INTERRUPTED!" : "CLIMB LOWERED!");
    }
}