package frc.robot.commands.swerve;

import static frc.robot.Constants.AutoAlignConstants.BLUE_STAGE_BACK_POSE;
import static frc.robot.Constants.AutoAlignConstants.BLUE_STAGE_LEFT_POSE;
import static frc.robot.Constants.AutoAlignConstants.BLUE_STAGE_RIGHT_POSE;
import static frc.robot.Constants.AutoAlignConstants.RED_STAGE_BACK_POSE;
import static frc.robot.Constants.AutoAlignConstants.RED_STAGE_LEFT_POSE;
import static frc.robot.Constants.AutoAlignConstants.RED_STAGE_RIGHT_POSE;

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

    /**
     * Generates a holonomic path from the robot to the nearest stage face. The robot should end up with its climb hooks
     * aligned directly above the chain. 
     *
     * @param swerveSubsystem The swerve drive to move the robot to its target.
     * @param isRed True if the robot is on the red alliance, false if on the blue alliance.
     * @return Pathfinding Command to bring the robot to its target position.
     */
    public static Command getStageAlignCommand(SwerveSubsystem swerveSubsystem, Boolean isRed) {
        Pose2d currentPose = swerveSubsystem.getRobotPosition();
        Pose2d targetPose = new Pose2d();
        double distance = Double.MAX_VALUE;

        Pose2d[] possibleTargets = isRed
            ? new Pose2d[]{RED_STAGE_BACK_POSE, RED_STAGE_LEFT_POSE, RED_STAGE_RIGHT_POSE}
            : new Pose2d[]{BLUE_STAGE_BACK_POSE, BLUE_STAGE_LEFT_POSE, BLUE_STAGE_RIGHT_POSE};

        for (Pose2d possibleTarget : possibleTargets) {
            double possibleDistance = currentPose.getTranslation().getDistance(possibleTarget.getTranslation());
            if (possibleDistance < distance) {
                distance = possibleDistance;
                targetPose = possibleTarget;
            }
        }

        return getAlignCommand(targetPose, swerveSubsystem);
    }
}