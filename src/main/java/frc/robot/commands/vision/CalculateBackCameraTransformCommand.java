package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Tries to get the transformation from the back left camera to the back right camera using a mutually visible
 * apriltag in Photonvision.
 */
public class CalculateBackCameraTransformCommand extends Command {
    private final PhotonCamera backLeft;
    private final PhotonCamera backRight;

    private PhotonTrackedTarget backLeftTarget;
    private PhotonTrackedTarget backRightTarget;
    private Transform3d backLeftToBackRight;

    /** Constructs a {@link CalculateBackCameraTransformCommand}. */
    public CalculateBackCameraTransformCommand(PhotonCamera backLeft, PhotonCamera backRight) {
        this.backLeft = backLeft;
        this.backRight = backRight;
        System.out.println("Starting . . .");
    }

    @Override
    public void execute() {
        PhotonPipelineResult backLeftResult = backLeft.getLatestResult();
        PhotonPipelineResult backRightResult = backRight.getLatestResult();
        backLeftTarget = backLeftResult.getBestTarget();
        backRightTarget = backRightResult.getBestTarget();

        System.out.println((backLeftTarget != null) + "   " + (backRightTarget != null));

        if (backLeftTarget == null || backRightTarget == null) {
            backLeftTarget = null;
            backRightTarget = null;
            return;
        }

        if (backLeftTarget.getFiducialId() != backRightTarget.getFiducialId()) {
            backLeftTarget = null;
            backRightTarget = null;
            return;
        }

        if (Math.abs(backRightResult.getTimestampSeconds() - backLeftResult.getTimestampSeconds()) > 0.05) {
            return;
        }
        
        backLeftToBackRight = backLeftTarget.getBestCameraToTarget()
                        .plus(backRightTarget.getBestCameraToTarget().inverse());
    }

    @Override
    public boolean isFinished() {
        return backLeftToBackRight != null;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            System.out.println(backLeftToBackRight);
        }
    }
}
