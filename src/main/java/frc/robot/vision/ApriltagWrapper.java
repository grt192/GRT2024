package frc.robot.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;

import static frc.robot.Constants.VisionConstants.*;

/**
 * The ApriltagWrapper class gets robot pose estimates from a given camera and publishes robot pose estimations over
 * NetworkTables.
 */
public class ApriltagWrapper {
    private final PhotonPoseEstimator poseEstimator;
    private final String name;

    private final BooleanSubscriber debugEnabled;
    private final DoublePublisher xPosPub;
    private final DoublePublisher yPosPub;
    private final DoublePublisher headingPub;

    /**
     * Constructs a PhotonVision connection to the coprocessor.
     * @param camera The PhotonVision camera object.
     * @param cameraPose The camera's 3d transformation relative to the robot.
     */
    public ApriltagWrapper(PhotonCamera camera, Transform3d cameraPose) {
        this.name = camera.getName();
        
        /* Pose Estimation Setup */
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        this.poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            cameraPose
        );
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        /* NetworkTables Setup */
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable poseTable = inst.getTable(POSE_TABLE_KEY);
        
        BooleanTopic debugEnabledTopic = poseTable.getBooleanTopic("debugEnabled");
        DoubleTopic xPosTopic = poseTable.getDoubleTopic("xPos" + name);
        DoubleTopic yPosTopic = poseTable.getDoubleTopic("yPos" + name);
        DoubleTopic headingTopic = poseTable.getDoubleTopic("heading" + name);

        this.debugEnabled = debugEnabledTopic.subscribe(false);
        this.xPosPub = xPosTopic.publish(PubSubOption.keepDuplicates(true));
        this.yPosPub = yPosTopic.publish(PubSubOption.keepDuplicates(true));
        this.headingPub = headingTopic.publish(PubSubOption.keepDuplicates(true));
    }

    /**
     * Get the estimated robot pose from a single photon pose estimator.
     * @param prevEstimatedRobotPose The last aggregate robot pose estimate.
     * @return The optional vision-estimated pose.
     */
    public Optional<EstimatedRobotPose> getRobotPose(Pose3d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> robotPose = poseEstimator.update();

        if (robotPose.isPresent() && debugEnabled.get())
            updateNetworkTables(robotPose.get().estimatedPose.toPose2d());

        return robotPose;
    }

    /**
     * Updates the camera's NetworkTables topics with a pose estimate.
     * @param robotPose The robot pose to update NetworkTables with.
     */
    private void updateNetworkTables(Pose2d robotPose) {
        xPosPub.set(robotPose.getX());
        yPosPub.set(robotPose.getY());
        headingPub.set(robotPose.getRotation().getDegrees());
    }
}
