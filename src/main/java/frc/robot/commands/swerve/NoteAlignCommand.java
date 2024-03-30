package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.vision.NoteDetectionWrapper;
import java.util.NoSuchElementException;

/** Rotates the robot to face directly towards a note on the ground. */
public class NoteAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final NoteDetectionWrapper noteDetector;
    private final BaseDriveController driveController;
    private final PIDController yawController;

    private double noteYawOffsetDegrees;
    private double angularVelocity;

    /**
     * Constructs a new {@link NoteAlignCommand}.
     *
     * @param swerveSubsystem The swerve subsystem to rotate the robot with.
     * @param noteDetector The note detector that will be used to identify and locate the note.
     */
    public NoteAlignCommand(SwerveSubsystem swerveSubsystem,
                            NoteDetectionWrapper noteDetector,
                            BaseDriveController driveController) {

        this.swerveSubsystem = swerveSubsystem;
        this.noteDetector = noteDetector;
        this.driveController = driveController;

        this.yawController = new PIDController(2.5, 0, 0);
        yawController.setSetpoint(0);
        yawController.setTolerance(1);
        // yawController.enableContinuousInput(-90, 90);

        angularVelocity = 0;

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
        return !driveController.getNoteAlign().getAsBoolean();
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

        angularVelocity = yawController.calculate(
            Units.degreesToRadians(noteYawOffsetDegrees) - angularVelocity * .1
        );

        swerveSubsystem.setRobotRelativeDrivePowers(
            .3,
            driveController.getLeftPower(),
            angularVelocity / SwerveSubsystem.MAX_OMEGA
        );
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended NoteAlignCommand");
    }
}
