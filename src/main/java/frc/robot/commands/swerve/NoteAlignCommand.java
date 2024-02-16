package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.NoSuchElementException;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;

public class NoteAlignCommand extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetectionWrapper noteDetector;

    private double noteYawOffsetDegrees;

    public NoteAlignCommand(SwerveSubsystem swerveSubsystem, NoteDetectionWrapper noteDetector) {
        this.swerveSubsystem = swerveSubsystem;
        this.noteDetector = noteDetector;
        System.out.println("Note Detector 3: " + noteDetector);

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Started NoteAlignCommand");

        this.noteYawOffsetDegrees = 0;
        try {
            noteYawOffsetDegrees = noteDetector.getNoteYawOffset().get();
        } catch(NoSuchElementException e) {
            System.out.println("Tried to align to a note, but none was detected.");
            this.end(true);
        }
        
        System.out.println("Rotating to note at offset " + this.noteYawOffsetDegrees + " degrees");

    }

    @Override
    public boolean isFinished() {
        return Math.abs(noteYawOffsetDegrees) < 1;
    }

    @Override
    public void execute() {
        try {
            noteYawOffsetDegrees = noteDetector.getNoteYawOffset().get();
        } catch(NoSuchElementException e) {

        }

        swerveSubsystem.setRobotRelativeDrivePowers(0.,0., -MathUtil.clamp(noteYawOffsetDegrees * .008, -.3, .3));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, 0);
        System.out.println("Ended NoteAlignCommand");
    }


}
