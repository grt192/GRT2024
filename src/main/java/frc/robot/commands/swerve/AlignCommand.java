package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/** Contains functions to create auto-alignment commands. */
public class AlignCommand {

    /**
     * Creates a PathPlanner align command using PathPlanner's pathfindToPose() and autoBuilder.
     *
     * @param targetPose The desired position for the robot at the end of the align command
     * @param swerveSubsystem The robot swerve subsystem to control
     * @return Pathfinding Command that pathfinds and aligns the robot
     */
    public static Command getAlignCommand(Pose2d targetPose, SwerveSubsystem swerveSubsystem) {
        Command command = AutoBuilder.pathfindToPose(
            targetPose, 
            new PathConstraints(
            2.0, 2.0, 
                    Units.degreesToRadians(720), Units.degreesToRadians(1080)
                    ), 
            0, 
            0.0
                );

        command.addRequirements(swerveSubsystem);
        
        return command;
    }

    /**
     * Creates a PathPlanner align command using PathPlanner's pathfindToPose() and autoBuilder.
     *
     * @param swerveSubsystem The robot swerve subsystem to control
     * @param isRed Whether or not we have a red alliance
     * @return Pathfinding Command that pathfinds and aligns the robot
     */
    public static Command getAmpAlignCommand(SwerveSubsystem swerveSubsystem, Boolean isRed) {
        
        Pose2d targetPose;

        if (isRed) {
            targetPose = AutoAlignConstants.RED_AMP_POSE;
        } else {
            targetPose = AutoAlignConstants.BLUE_AMP_POSE;
        }

        Command command = AutoBuilder.pathfindToPose(
            targetPose, 
            new PathConstraints(
            2.0, 2.0, 
                    Units.degreesToRadians(720), Units.degreesToRadians(1080)
                    ), 
            0, 
            0.0
                );

        command.addRequirements(swerveSubsystem);
        
        return command;
    }
}