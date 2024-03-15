package frc.robot.util.bindings;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.SwerveStopCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;

/** Control bindings for swerve. */
public class SwerveBindings {
    private final SwerveSubsystem swerveSubsystem;
    private final BaseDriveController driveController;
    private final FieldManagementSubsystem fmsSubsystem;
    private final LightBarSubsystem lightBarSubsystem;

    /** Control bindings for swerve. */
    public SwerveBindings(
        SwerveSubsystem swerveSubsystem, 
        BaseDriveController driveController,
        FieldManagementSubsystem fmsSubsystem,
        LightBarSubsystem lightBarSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.driveController = driveController;
        this.fmsSubsystem = fmsSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
    }

    /** Sets the bindings. */
    public void setBindings() {
        /* SWERVE BINDINGS */

        /* Shooter Aim -- Holding down the button will change the shooter's pitch to aim it at the speaker. */
        // drive

        /* Amp Align -- Pressing and holding the button will cause the robot to automatically path find to the amp.
         * Releasing the button will stop the robot (and the path finding). */
        driveController.getAmpAlign().onTrue(new InstantCommand(
            () -> lightBarSubsystem.setLightBarStatus(LightBarStatus.AUTO_ALIGN)
            ).andThen(new ParallelRaceGroup(
                AlignCommand.getAmpAlignCommand(swerveSubsystem, fmsSubsystem.isRedAlliance()),
                new ConditionalWaitCommand(
                    () -> !driveController.getAmpAlign().getAsBoolean())
        )));

        /* Note align -- deprecated, new version in the works*/
        // driveController.getNoteAlign().onTrue(new ParallelRaceGroup(
        //         new AutoIntakeSequence(elevatorSubsystem, intakeRollerSubsystem, swerveSubsystem, noteDetector,
        //                 lightBarSubsystem)
        //                 .unless(() -> noteDetector.getNote().isEmpty()),
        //         new ConditionalWaitCommand(() -> !driveController.getNoteAlign().getAsBoolean())));
    
        /* Swerve Stop -- Pressing the button completely stops the robot's motion. */
        driveController.getSwerveStop().onTrue(new SwerveStopCommand(swerveSubsystem));

        /* Driving -- One joystick controls translation, the other rotation. If the robot-relative button is held down,
         * the robot is controlled along its own axes, otherwise controls apply to the field axes by default. If the
         * swerve aim button is held down, the robot will rotate automatically to always face a target, and only
         * translation will be manually controllable. */
        swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (driveController.getRelativeMode()) {
                swerveSubsystem.setRobotRelativeDrivePowers(
                        driveController.getForwardPower(),
                        driveController.getLeftPower(),
                        driveController.getRotatePower()
                );
            } else {
                if (driveController.getSwerveAimMode()) {
                    swerveSubsystem.setSwerveAimDrivePowers(
                            driveController.getForwardPower(),
                            driveController.getLeftPower());
                } else {
                    swerveSubsystem.setDrivePowers(
                            driveController.getForwardPower(),
                            driveController.getLeftPower(),
                            driveController.getRotatePower());
                }
            }
        }, swerveSubsystem));

        /* Pressing the button resets the field axes to the current robot axes. */
        driveController.getDriverHeadingResetButton().onTrue(new InstantCommand(() -> {
            swerveSubsystem.resetDriverHeading();
        }));
    }
}
