package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class NoteDetectionWrapper {
    private final PhotonCamera camera;

    private final BooleanPublisher noteDetectedPub;

    public NoteDetectionWrapper(PhotonCamera camera) {
        this.camera = camera;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = inst.getTable(VISION_TABLE_KEY);

        BooleanTopic noteDetectedTopic = visionTable.getBooleanTopic("noteDetected");
        this.noteDetectedPub = noteDetectedTopic.publish(PubSubOption.keepDuplicates(true));
    }

    public Optional<PhotonTrackedTarget> getNote() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            noteDetectedPub.set(false);
            return Optional.empty();
        }

        PhotonTrackedTarget target = result.getBestTarget();
        
        noteDetectedPub.set(true);
        return Optional.of(target);
    }
    
}
