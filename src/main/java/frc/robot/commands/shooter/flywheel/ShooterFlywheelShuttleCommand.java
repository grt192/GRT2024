package frc.robot.commands.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Pose2dSupplier;

/** Shuttles notes. */
public class ShooterFlywheelShuttleCommand extends Command {

    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double ERROR_MULTIPLIER = 0.08;
    private static final double TARGET_ANGLE = 30;

    private final SwerveSubsystem swerveSubsystem;
    private final ShooterFlywheelSubsystem flywheelSubsystem;
    private Pose2dSupplier poseSupplier; //new Pose2d();
    private final ShooterPivotSubsystem pivotSubsystem;

    private boolean redAlliance = true;
    private double angleOffset;

    /** Constructor for Shuttle Command. */
    public ShooterFlywheelShuttleCommand(SwerveSubsystem swerveSubsystem, 
           ShooterFlywheelSubsystem flywheelSubsystem, Pose2dSupplier poseSupplier, 
           ShooterPivotSubsystem pivotSubsystem) {

        this.swerveSubsystem = swerveSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.poseSupplier = poseSupplier;
        this.pivotSubsystem = pivotSubsystem;

        addRequirements(swerveSubsystem, flywheelSubsystem);
    }


    private double getAngle(double targetX, double targetY, double robotX, double robotY) {
        return Math.atan((targetY - robotY) / (targetX - robotX));
    }

    /** Runs command on init. */
    public void initialize() {
        Pose2d currentField = poseSupplier.getPose2d();

        //red alliance aim pos (652.73-114, 323-96)
        double redAllianceX = Units.inchesToMeters(652.73 - 114);
        double redAllianceY = Units.inchesToMeters(323 - 96);

        //blue alliance aim pos (114, 323-96)
        double blueAllianceX = Units.inchesToMeters(114);
        double blueAllianceY = Units.inchesToMeters(323 - 96);

        double robotX = currentField.getX();
        double robotY = currentField.getY();
        double robotAngle = currentField.getRotation().getDegrees();

        if (redAlliance) {
            angleOffset = getAngle(redAllianceX, redAllianceY, robotX, robotY) - robotAngle;
        } else {
            angleOffset = 180 - getAngle(blueAllianceX, blueAllianceY, robotX, robotY) - robotAngle;
        }

        System.out.println("Angle Offset is: " + angleOffset);
    }

    /** Is robot in center line. */
    public boolean isFinished() {

        return (poseSupplier.getPose2d().getX() < 326.365 + 70 || poseSupplier.getPose2d().getX() > 326.36 - 70);
    }

    /** What the command has to execute. */
    public void execute() {

        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, -MathUtil.clamp(
            angleOffset * ERROR_MULTIPLIER, -MAX_ROTATION_POWER, MAX_ROTATION_POWER)); //CHANGE ANGULAR POWER
        flywheelSubsystem.setShooterMotorSpeed(ShooterConstants.FLYWHEEL_SHUTTLE_SPEED);
        pivotSubsystem.setAngle(TARGET_ANGLE);
        
    }

    /** Run after command has ended. Turns of shooter and swerve speed.*/
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotRelativeDrivePowers(0, 0, 0);
        flywheelSubsystem.stopShooter();

        System.out.println("Ended shuttle command");
    }
}
