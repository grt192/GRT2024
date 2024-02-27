package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignCommand {

    /**
     * Create a PathPlanner align command using PathPlanner's pathfindToPose() and autoBuilder.
     *
     * @param targetPose The desired position for the robot at the end of the align command
     * @param swerveSubsystem The robot swerve subsystem to control
     * 
     * @return Pathfinding Command that pathfinds and aligns the robot
     * 
     */
    public static Command getAlignCommand(Pose2d targetPose, SwerveSubsystem swerveSubsystem){
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
