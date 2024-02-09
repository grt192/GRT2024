package frc.robot.commands.align;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignCommand {
    
    public AlignCommand(){


    }

    static public Command getAlignCommand(Pose2d targetPose){

        return  AutoBuilder.pathfindToPose(
            targetPose, 
            new PathConstraints(
        4.0, 4.0, 
                Units.degreesToRadians(360), Units.degreesToRadians(540)
            ), 
    0, 
    2.0
        );
    }

}
