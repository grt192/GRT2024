package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbLowerCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    public ClimbLowerCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("LOWERING CLIMB...");
        climbSubsystem.goToExtension(0);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtExtension();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted ? "CLIMB LOWERING INTERRUPTED!" : "CLIMB LOWERED!");
    }

}
