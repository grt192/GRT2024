package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import static frc.robot.Constants.ClimbConstants.*;

public class ClimbRaiseCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    public ClimbRaiseCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("RAISING CLIMB...");
        climbSubsystem.goToExtension(RAISE_LIMIT_METERS);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isAtTargetExtension();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println(interrupted ? "CLIMB RAISING INTERRUPTED!" : "CLIMB RAISED!");
    }

}
