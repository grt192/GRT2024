package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.NoSuchElementException;

import org.photonvision.PhotonCamera;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

public class NoteAlignCommand extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetectionWrapper noteDetector;

    private Rotation2d targetHeading;

    public NoteAlignCommand(SwerveSubsystem swerveSubsystem, PhotonCamera camera) {
        this.swerveSubsystem = swerveSubsystem;
        noteDetector = new NoteDetectionWrapper(camera);

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        double noteYawOffsetDegrees = 0;
        try {
            noteYawOffsetDegrees = noteDetector.getNoteYawOffset().get();
        } catch(NoSuchElementException e) {
            System.out.println("Tried to align to a note, but none was detected.");
            this.end(true);
        }
        
        this.targetHeading = swerveSubsystem.getDriverHeading().plus(Rotation2d.fromDegrees(noteYawOffsetDegrees));

        swerveSubsystem.setDrivePowerswithHeadingLock(0, 0, targetHeading.getDegrees());

    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.getDriverHeading() == this.targetHeading;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        
    }


}
