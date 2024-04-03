package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

/** Lowers climb fully when off the chain. Should run at the start of auton to ensure that both arms are at their lowest
 *  positions so that the robot may fit underneath the stage. */
public class ClimbLowerCommand extends Command {
    private static final double LOWERING_SPEED = -0.9;
    private final ClimbSubsystem climbSubsystem;
    
    /** Constructs a new {@link ClimbLowerCommand}. */
    public ClimbLowerCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setSpeeds(LOWERING_SPEED, LOWERING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isLowered();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
