package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.pivot.IntakePivotVerticalCommand;
import frc.robot.commands.shooter.feed.ShooterFeedShootCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterFeederSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;

public class ShootModeCommand extends SequentialCommandGroup {
    IntakePivotSubsystem intakePivotSubsystem;
    ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    ShooterFeederSubsystem shooterFeederSubsystem;

    public ShootModeCommand(){
        addCommands(new IntakePivotVerticalCommand(intakePivotSubsystem).alongWith(new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem)), 
                    new ShooterFeedShootCommand(shooterFeederSubsystem));
    }
}
