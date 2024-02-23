package frc.robot.vision;

import static frc.robot.Constants.VisionConstants.VISION_TABLE_KEY;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Detects notes' positions withing a given camera frame over PhotonVision and publishes note detections over
 * NetworkTables.
 */
public class NoteDetectionWrapper {
    private final PhotonCamera camera;

    private final BooleanPublisher noteDetectedPub;

    /**
     * Constructs a {@link NoteDetectionWrapper}. 
     *
     * @param camera The PhotonVision camera object.
     */
    public NoteDetectionWrapper(PhotonCamera camera) {
        this.camera = camera;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = inst.getTable(VISION_TABLE_KEY);

        BooleanTopic noteDetectedTopic = visionTable.getBooleanTopic("noteDetected");
        this.noteDetectedPub = noteDetectedTopic.publish(PubSubOption.keepDuplicates(true));
    }

    /**
     * Tries to detect a note. If successful, it will return information about its position within the frame and update
     * the {@code noteDetected} boolean over NetworkTables. 
     *
     * @return Optionally, a note's {@link PhotonTrackedTarget} object.
     */
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
