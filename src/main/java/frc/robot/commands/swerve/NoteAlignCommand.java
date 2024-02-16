package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
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
    private double noteYawOffsetDegrees;

    public NoteAlignCommand(SwerveSubsystem swerveSubsystem, PhotonCamera camera) {
        this.swerveSubsystem = swerveSubsystem;
        noteDetector = new NoteDetectionWrapper(camera);

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.noteYawOffsetDegrees = 0;
        try {
            noteYawOffsetDegrees = noteDetector.getNoteYawOffset().get();
        } catch(NoSuchElementException e) {
            System.out.println("Tried to align to a note, but none was detected.");
            this.end(true);
        }
        
        System.out.println("Note Yaw Offset:" + this.noteYawOffsetDegrees + " degrees");

        this.targetHeading = swerveSubsystem
            .getRobotPosition().getRotation()
            .minus(Rotation2d.fromDegrees(noteYawOffsetDegrees));

    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerveSubsystem.getRobotPosition().getRotation().getDegrees() - this.targetHeading.getDegrees()) < 1;
    }

    @Override
    public void execute() {
        try {
            noteYawOffsetDegrees = noteDetector.getNoteYawOffset().get();
            this.targetHeading = swerveSubsystem
            .getRobotPosition().getRotation()
            .minus(Rotation2d.fromDegrees(noteYawOffsetDegrees));
            System.out.println("Note Yaw Offset: " + noteYawOffsetDegrees + " degrees");
        } catch(NoSuchElementException e) {

        }

        swerveSubsystem.setRobotRelativeDrivePowers(0.,0., -MathUtil.clamp(noteYawOffsetDegrees * .01, -.3, .3));

        // swerveSubsystem.setDrivePowerswithHeadingLock(0, 0, targetHeading.getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setDrivePowers(0, 0, 0);
        System.out.println("Ended NoteAlignCommand");
    }


}
