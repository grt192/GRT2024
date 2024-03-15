// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.AutonBuilder;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.climb.ManualClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.bindings.MishaBindings;
import frc.robot.util.bindings.SwerveBindings;


/** The robot container. */
public class RobotContainer {
    private final BaseDriveController driveController;
    private final SwerveSubsystem swerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();

    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;

    private final ManualClimbSubsystem climbSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem();
    private final LightBarSubsystem lightBarSubsystem = new LightBarSubsystem();

    private final SendableChooser<Command> autonPathChooser;
    private final AutonBuilder autonBuilder;

    /* MECH BUTTONS */
    private final XboxController mechController = new XboxController(2);

    private final GenericHID switchboard = new GenericHID(3);

    private UsbCamera driverCamera;
    private MjpegServer driverCameraServer;

    ChoreoTrajectory trajectory;

    private final ShuffleboardTab swerveCrauton;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem();
        swerveSubsystem.setVerbose(false); // SET THIS TO true FOR TUNING VALUES

        intakePivotSubsystem = new IntakePivotSubsystem();

        shooterPivotSubsystem = new ShooterPivotSubsystem(swerveSubsystem::getRobotPosition);
        shooterFlywheelSubsystem = new ShooterFlywheelSubsystem(swerveSubsystem::getRobotPosition);

        elevatorSubsystem = new ElevatorSubsystem();

        climbSubsystem = new ManualClimbSubsystem();

        swerveCrauton = Shuffleboard.getTab("Auton");

        if (DriverStation.getJoystickName(0).equals("Controller (Xbox One For Windows)")) {
            driveController = new XboxDriveController();
        } else {
            driveController = new DualJoystickDriveController();
        }

        driverCamera = new UsbCamera("fisheye", 0);
        driverCamera.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
        driverCameraServer = new MjpegServer("m1", 1181);
        driverCameraServer.setSource(driverCamera);

        autonBuilder = new AutonBuilder(
            intakePivotSubsystem, intakeRollerSubsystem, 
            shooterFlywheelSubsystem, shooterPivotSubsystem, 
            elevatorSubsystem, 
            swerveSubsystem, 
            lightBarSubsystem, fmsSubsystem
        );

        autonPathChooser = new SendableChooser<>();
        autonPathChooser.setDefaultOption("topPreloaded", autonBuilder.getMiddleFourPiece());
        autonPathChooser.addOption("top2Piece", autonBuilder.getTopTwoPiece());
        autonPathChooser.addOption("middlePreloaded", autonBuilder.getMiddlePreloaded());
        autonPathChooser.addOption("middle2Piece", autonBuilder.getMiddleTwoPiece());
        autonPathChooser.addOption("middle3Piece", autonBuilder.getMiddleThreePiece());
        autonPathChooser.addOption("middle4Piece", autonBuilder.getMiddleFourPiece());
        autonPathChooser.addOption("bottomPreloaded", autonBuilder.getBottomPreloaded());
        autonPathChooser.addOption("bottom2Piece", autonBuilder.getBottomTwoPiece());
        autonPathChooser.addOption("bottomDisruptor", autonBuilder.getBottomDisruptor());

        swerveCrauton.add(autonPathChooser);

        configureBindings();
    }

    private void configureBindings() {        
        MishaBindings mechBindings = new MishaBindings(
            mechController, switchboard, lightBarSubsystem, 
            intakePivotSubsystem, intakeRollerSubsystem, 
            elevatorSubsystem, 
            shooterPivotSubsystem, shooterFlywheelSubsystem, 
            climbSubsystem
        );

        SwerveBindings swerveBindings = new SwerveBindings(
            swerveSubsystem, driveController, 
            fmsSubsystem, lightBarSubsystem
        );

        mechBindings.setAllBindings(); 
        swerveBindings.setBindings();       
    }

    /**
     * Returns the autonomous command.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return autonPathChooser.getSelected();
    }
}