package frc.robot.commands.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Pose2dSupplier;

/** Shuttles notes. */
public class ShooterFlywheelShuttleCommand extends Command {

    private static final double MAX_ROTATION_POWER = 0.3;
    private static final double ERROR_MULTIPLIER = 0.08;
    private static final double TARGET_ANGLE = Units.degreesToRadians(42);
    
    private final Translation2d BLUE_SHUTTLE_POINT = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(300));
    private final Translation2d RED_SHUTTLE_POINT = new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(300));

    private final SwerveSubsystem swerveSubsystem;
    private final ShooterFlywheelSubsystem flywheelSubsystem;
    private final ShooterPivotSubsystem pivotSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private final double shooterSpeed;
    private XboxController controller;


    private boolean redAlliance = true;
    private double angleOffset;

    /** Constructor for Shuttle Command. */
    public ShooterFlywheelShuttleCommand(SwerveSubsystem swerveSubsystem, 
           ShooterFlywheelSubsystem flywheelSubsystem, FieldManagementSubsystem fmsSubsystem, 
           ShooterPivotSubsystem pivotSubsystem, double speed, XboxController mechController) {

        this.swerveSubsystem = swerveSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        this.shooterSpeed = speed;
        this.controller = mechController;

        addRequirements(flywheelSubsystem, pivotSubsystem);
    }


    /** Runs command on init. */
    public void initialize() {
        swerveSubsystem.setTargetPoint(fmsSubsystem.isRedAlliance() ? RED_SHUTTLE_POINT : BLUE_SHUTTLE_POINT);
        swerveSubsystem.setAim(true);
    }
    /** What the command has to execute. */
    public void execute() {
        flywheelSubsystem.setShooterMotorSpeed(shooterSpeed);
        pivotSubsystem.setAngle(TARGET_ANGLE);
        if(flywheelSubsystem.atSpeed()) {
            controller.setRumble(RumbleType.kBothRumble, .4);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }

    /** Run after command has ended. Turns of shooter and swerve speed.*/
    public void end(boolean interrupted) {
        flywheelSubsystem.stopShooter();
        swerveSubsystem.setAim(false);

        System.out.println("Ended shuttle command");
    }
}
