package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants;
// import frc.robot.util.ShuffleboardUtil;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A connection to PhotonVision on the coprocessor.
 */
public class PhotonWrapper {
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;

    // private final ShuffleboardTab shuffleboardTab;
    // private final GenericEntry frontStatusEntry, xPosFrontEntry, yPosFrontEntry, timestampFrontEntry;
    // private final GenericEntry backStatusEntry, xPosBackEntry, yPosBackEntry, timestampBackEntry;

    // // Whether to read and update shuffleboard values
    // private static final boolean OVERRIDE_SHUFFLEBOARD_ENABLE = false;
    // private volatile boolean SHUFFLEBOARD_ENABLE = OVERRIDE_SHUFFLEBOARD_ENABLE || Constants.GLOBAL_SHUFFLEBOARD_ENABLE;

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     */
    public PhotonWrapper() {
        try {
            // AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");

            frontPoseEstimator = new PhotonPoseEstimator(
                // new AprilTagFieldLayout(AprilTagFields.k2023ChargedUp.m_resourceFile),
                // new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json"),
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                FRONT_CAMERA,
                FRONT_CAMERA_POSE
            );
            frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

            backPoseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                RIGHT_CAMERA,
                RIGHT_CAMERA_POSE
            );
            backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // shuffleboardTab = Shuffleboard.getTab("PhotonVision");

        // frontStatusEntry = shuffleboardTab.add("Front tag detected", false).withPosition(0, 0).getEntry();
        // xPosFrontEntry = shuffleboardTab.add("Front x-pos", 0).withPosition(0, 1).getEntry();
        // yPosFrontEntry = shuffleboardTab.add("Front y-pos", 0).withPosition(1, 1).getEntry();
        // timestampFrontEntry = shuffleboardTab.add("Front timestamp", 0).withPosition(2, 1).getEntry();

        // backStatusEntry = shuffleboardTab.add("Back tag detected", false).withPosition(0, 2).getEntry();
        // xPosBackEntry = shuffleboardTab.add("Back x-pos", 0).withPosition(0, 3).getEntry();
        // yPosBackEntry = shuffleboardTab.add("Back y-pos", 0).withPosition(1, 3).getEntry();
        // timestampBackEntry = shuffleboardTab.add("Back timestamp", 0).withPosition(2, 3).getEntry();

        // GenericEntry shuffleboardEnableEntry = shuffleboardTab.add("Shuffleboard enable", SHUFFLEBOARD_ENABLE)
        //     .withPosition(4, 0)
        //     .withWidget(BuiltInWidgets.kToggleSwitch)
        //     .getEntry();
        // ShuffleboardUtil.addBooleanListener(shuffleboardEnableEntry, (value) -> SHUFFLEBOARD_ENABLE = value);
    }

    /**
     * Get the optional estimated robot pose from a single photon pose estimator. 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate.
     * @param poseEstimator The pose estimator to update.
     * @return The estimated optional estimated vision pose.
     */
    public Optional<EstimatedRobotPose> getRobotPose(Pose2d prevEstimatedRobotPose, PhotonPoseEstimator poseEstimator) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose); // input odometry pose
        return poseEstimator.update();
    }

    /**
     * Gets the estimated robot poses from vision. Call this every periodic loop to update the drivetrain pose 
     * estimator in `SwerveSubsystem` with vision data.
     * 
     * @param prevEstimatedRobotPose The last odometry robot pose estimate, for setting the vision reference pose.
     * @return A list of estimated vision poses.
     */
    public List<EstimatedRobotPose> getRobotPoses(Pose2d prevEstimatedRobotPose) {
        Optional<EstimatedRobotPose> frontEstimate = getRobotPose(prevEstimatedRobotPose, frontPoseEstimator);
        Optional<EstimatedRobotPose> backEstimate = getRobotPose(prevEstimatedRobotPose, backPoseEstimator);

        // // Update Shuffleboard
        // if (SHUFFLEBOARD_ENABLE) {
        //     frontStatusEntry.setBoolean(frontEstimate.isPresent());
        //     backStatusEntry.setBoolean(backEstimate.isPresent());

        //     frontEstimate.ifPresent((frontPose) -> {
        //         xPosFrontEntry.setValue(Units.metersToInches(frontPose.estimatedPose.getX()));
        //         yPosFrontEntry.setValue(Units.metersToInches(frontPose.estimatedPose.getY()));
        //         timestampFrontEntry.setValue(frontPose.timestampSeconds);
        //     });

        //     backEstimate.ifPresent((backPose) -> {
        //         xPosBackEntry.setValue(Units.metersToInches(backPose.estimatedPose.getX()));
        //         yPosBackEntry.setValue(Units.metersToInches(backPose.estimatedPose.getY()));
        //         timestampBackEntry.setValue(backPose.timestampSeconds);
        //     });
        // }

        List<EstimatedRobotPose> outputPoses = new ArrayList<EstimatedRobotPose>();
        if (frontEstimate.isPresent()) outputPoses.add(frontEstimate.get());
        if (backEstimate.isPresent()) outputPoses.add(backEstimate.get());
        return outputPoses;
    }
}
