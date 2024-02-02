package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbIdleCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    public ClimbIdleCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("RESETTING CLIMB...");
        climbSubsystem.goToExtension(0);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtTargetExtension();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted ? "CLIMB RESETTING INTERRUPTED!" : "CLIMB RESET TO IDLE!");
    }

}
