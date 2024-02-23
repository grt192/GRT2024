package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;
import java.util.NoSuchElementException;

/**
 *  Rotates the robot to face directly towards a note on the ground.
 */
public class NoteAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetectionWrapper noteDetector;

    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double ERROR_MULTIPLIER = 0.08;
    private static final double ANGULAR_TOLERANCE_DEGREES = 1;

    private double noteYawOffsetDegrees;

    /**
     * Constructs a new {@link NoteAlignCommand}.
     *
     * @param swerveSubsystem The swerve subsystem to rotate the robot with.
     * @param noteDetector The note detector that will be used to identify and locate the note.
     */
    public NoteAlignCommand(SwerveSubsystem swerveSubsystem, NoteDetectionWrapper noteDetector) {
        this.swerveSubsystem = swerveSubsystem;
        this.noteDetector = noteDetector;

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Started NoteAlignCommand");

        this.noteYawOffsetDegrees = 0;
        try {
            noteYawOffsetDegrees = noteDetector.getNote().get().getYaw();
        } catch (NoSuchElementException e) {
            System.out.println("Tried to align to a note, but none was detected.");
            this.end(true);
        }
        
        System.out.println("Rotating to note at offset " + this.noteYawOffsetDegrees + " degrees");

    }

    @Override
    public boolean isFinished() {
        return Math.abs(noteYawOffsetDegrees) < ANGULAR_TOLERANCE_DEGREES;
    }

    @Override
    public void execute() {
        try {
            noteYawOffsetDegrees = noteDetector.getNote().get().getYaw();
        } catch (NoSuchElementException e) {
            /* It's usually fine if the note can't be detected for a few cycles, as long as it is detected again soon
            after. If it remains out of frame, this issue should be rather evident (the robot will spin in place) and
            is probably not worth logging. */
        }

        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, -MathUtil.clamp(
            noteYawOffsetDegrees * ERROR_MULTIPLIER, -MAX_ROTATION_POWER, MAX_ROTATION_POWER)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, 0);
        System.out.println("Ended NoteAlignCommand");
    }


}
