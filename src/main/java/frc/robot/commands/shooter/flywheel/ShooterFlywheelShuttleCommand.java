package frc.robot.commands.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Pose2dSupplier;

/** Shuttles notes. */
public class ShooterFlywheelShuttleCommand extends Command {

    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double ANGULAR_TOLERANCE_DEGREES = 1;
    private static final double ERROR_MULTIPLIER = 0.08;

    private final SwerveSubsystem swerveSubsystem;
    private final ShooterFlywheelSubsystem flywheelSubsystem;
    private Pose2dSupplier poseSupplier; //new Pose2d();

    private boolean redAlliance = true;
    private double angleOffset;

    /** Constructor for Shuttle Command. */
    public ShooterFlywheelShuttleCommand(SwerveSubsystem swerveSubsystem, 
           ShooterFlywheelSubsystem flywheelSubsystem, Pose2dSupplier poseSupplier) {

        this.swerveSubsystem = swerveSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.poseSupplier = poseSupplier;

        addRequirements(swerveSubsystem, flywheelSubsystem);
    }

    /** Runs command on init. */
    public void initialize() {
        Pose2d currentField = poseSupplier.getPose2d();

        if (redAlliance) {
            angleOffset = currentField.getRotation().getDegrees();
        } else {
            angleOffset = -1 * (currentField.getRotation().getDegrees() - 180);
        }

        System.out.println("Starting shuttle command");
    }

    /** Under what condition the command must end. */
    public boolean isFinished() {
        return Math.abs(angleOffset) < ANGULAR_TOLERANCE_DEGREES;
    }

    /** What the command has to execute. */
    public void execute() {
        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, -MathUtil.clamp(
            angleOffset * ERROR_MULTIPLIER, -MAX_ROTATION_POWER, MAX_ROTATION_POWER)); //CHANGE ANGULAR POWER
        flywheelSubsystem.setShooterMotorSpeed(ShooterConstants.FLYWHEEL_SHUTTLE_SPEED);
    }

    /** Run after command has ended. Turns of shooter and swerve speed.*/
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, 0);
        flywheelSubsystem.stopShooter();

        System.out.println("Ended shuttle command");
    }
}
