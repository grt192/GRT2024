package frc.robot.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class NoteDetectionWrapper {
    private final PhotonCamera camera;

    public NoteDetectionWrapper(PhotonCamera camera) {
        this.camera = camera;
    }

    public Optional<Double> getNoteYawOffset() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        PhotonTrackedTarget target = result.getBestTarget();
        
        return Optional.of(target.getYaw());
    }
    
}
