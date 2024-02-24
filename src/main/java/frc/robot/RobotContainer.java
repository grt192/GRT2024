// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.NotePosition;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.commands.IdleCommand;
import frc.robot.commands.climb.ClimbLowerCommand;
import frc.robot.commands.climb.ClimbRaiseCommand;
import frc.robot.commands.elevator.ElevatorToAMPCommand;
import frc.robot.commands.elevator.ElevatorToChuteCommand;
import frc.robot.commands.elevator.ElevatorToGroundCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOutakeCommand;
import frc.robot.commands.sequences.AutoIntakeSequence;
import frc.robot.commands.sequences.ShootModeSequence;
import frc.robot.commands.shooter.pivot.ShooterPivotSetAngleCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotVerticalCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.commands.swerve.SwerveStopCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;
import frc.robot.vision.NoteDetectionWrapper;

import static frc.robot.Constants.VisionConstants.NOTE_CAMERA;

import java.sql.Driver;
import java.util.function.BooleanSupplier;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final BaseDriveController driveController;
    private final BaseSwerveSubsystem baseSwerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollerSubsystem = new IntakeRollersSubsystem();

    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;

    private final ClimbSubsystem climbSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final NoteDetectionWrapper noteDetector;

    // Configure the trigger bindings

    private final XboxController mechController = new XboxController(2);
    private final JoystickButton aButton = new JoystickButton(mechController, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(mechController, XboxController.Button.kB.value);
    private final JoystickButton leftBumper = new JoystickButton(mechController,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(mechController,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton xButton = new JoystickButton(mechController, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(mechController, XboxController.Button.kY.value);

    private final GenericHID switchboard = new GenericHID(3);
    private final JoystickButton redButton = new JoystickButton(switchboard, 5);

    private UsbCamera camera1;
    private MjpegServer mjpgserver1;
    // private final JoystickButton xButton = new JoystickButton(mechController,
    // XboxController.Button.kX.value);

    ChoreoTrajectory traj;
    // private final SwerveModule module;

    private PIDController xPID;
    private PIDController yPID;

    private final GenericEntry xError, yError;

    private final ShuffleboardTab swerveCrauton;

    private final BooleanSupplier isRed;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // construct Test
        // module = new SwerveModule(6, 7, 0);
        // baseSwerveSubsystem = new TestSingleModuleSwerveSubsystem(module);
        isRed = () -> false; // DriverStation.getAlliance() == new Optional<Alliance> ;
        baseSwerveSubsystem = new SwerveSubsystem();
        intakePivotSubsystem = new IntakePivotSubsystem();

        shooterPivotSubsystem = new ShooterPivotSubsystem(isRed.getAsBoolean(), baseSwerveSubsystem::getRobotPosition);
        shooterFlywheelSubsystem = new ShooterFlywheelSubsystem();

        climbSubsystem = new ClimbSubsystem();

        elevatorSubsystem = new ElevatorSubsystem();

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);

        traj = Choreo.getTrajectory("2mLine");

        swerveCrauton = Shuffleboard.getTab("Auton");

        xError = swerveCrauton.add("Xerror", 0).withPosition(8, 0).getEntry();
        yError = swerveCrauton.add("Yerror", 0).withPosition(9, 0).getEntry();
        if (DriverStation.getJoystickName(0).equals("Cyborg V.1")) {
            driveController = new DualJoystickDriveController();
        } else {
            driveController = new XboxDriveController();
        }
        // private final SwerveModule module;
        // construct Test
        // module = new SwerveModule(6, 7, 0);
        // baseSwerveSubsystem = new TestSingleModuleSwerveSubsystem(module);
        // baseSwerveSubsystem = new SwerveSubsystem();
        // intakePivotSubsystem = new IntakePivotSubsystem();

        noteDetector = new NoteDetectionWrapper(NOTE_CAMERA);

        camera1 = new UsbCamera("camera1", 0);
        camera1.setFPS(60);
        camera1.setBrightness(45);
        camera1.setResolution(176, 144);
        mjpgserver1 = new MjpegServer("m1", 1181);
        mjpgserver1.setSource(camera1);

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        aButton.onTrue(new ElevatorToChuteCommand(elevatorSubsystem).andThen(
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem).andThen(
                        new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.1))));

        bButton.onTrue(new IdleCommand(intakePivotSubsystem, intakeRollerSubsystem,
                elevatorSubsystem,
                shooterPivotSubsystem, shooterFlywheelSubsystem,
                climbSubsystem, ledSubsystem));

        leftBumper.onTrue(new ShootModeSequence(intakeRollerSubsystem,
                elevatorSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem,
                ledSubsystem).andThen(
                        new ConditionalWaitCommand(() -> mechController.getRightTriggerAxis() > .1)));

        rightBumper.onTrue(new ElevatorToAMPCommand(elevatorSubsystem).andThen(
                new InstantCommand(() -> ledSubsystem.setNoteMode(NotePosition.INTAKE_READY_TO_SHOOT)),
                new ConditionalWaitCommand(() -> mechController.getRightTriggerAxis() > .1),
                new IntakeRollerOutakeCommand(intakeRollerSubsystem),
                new InstantCommand(() -> ledSubsystem.setNoteMode(NotePosition.NONE))));

        xButton.onTrue(new IntakeRollerOutakeCommand(intakeRollerSubsystem).andThen(
                new InstantCommand(() -> ledSubsystem.setNoteMode(NotePosition.NONE))));

        if (baseSwerveSubsystem instanceof SwerveSubsystem) {
            final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;
            swerveCrauton.add("AUTO ALIGN BLUE AMP",
                    AlignCommand.getAlignCommand(AutoAlignConstants.BLUE_AMP_POSE, swerveSubsystem));

            ledSubsystem.setDefaultCommand(new RunCommand(() -> {
                ledSubsystem.setDriverHeading(
                        driveController.getRelativeMode() ? 0 : -swerveSubsystem.getDriverHeading().getRadians());
                ledSubsystem.setNoteSeen(noteDetector.getNote().isPresent());
            }, ledSubsystem));

            driveController.getAmpAlign().onTrue(new ParallelRaceGroup(
                    AlignCommand.getAlignCommand(AutoAlignConstants.BLUE_AMP_POSE, swerveSubsystem),
                    new ConditionalWaitCommand(() -> !driveController.getAmpAlign().getAsBoolean())));

            driveController.getNoteAlign().onTrue(new ParallelRaceGroup(
                    new AutoIntakeSequence(elevatorSubsystem, intakeRollerSubsystem, swerveSubsystem, noteDetector,
                            ledSubsystem)
                            .unless(() -> noteDetector.getNote().isEmpty()),
                    new ConditionalWaitCommand(() -> !driveController.getNoteAlign().getAsBoolean())));
            driveController.getSwerveStop().onTrue(new SwerveStopCommand(swerveSubsystem));

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                if (driveController.getRelativeMode()) {
                    swerveSubsystem.setRobotRelativeDrivePowers(driveController.getForwardPower(),
                            driveController.getLeftPower(), driveController.getRotatePower());
                } else {
                  if(driveController.getTurnMode()){
                    swerveSubsystem.setAimMode(driveController.getForwardPower(), driveController.getLeftPower());
                  }
                  else{
                    swerveSubsystem.setDrivePowers(driveController.getForwardPower(), driveController.getLeftPower(), driveController.getRotatePower());
                  }
                }
                // pivotSubsystem.setFieldPosition(swerveSubsystem.getRobotPosition());
                xError.setValue(xPID.getPositionError());
                yError.setValue(yPID.getPositionError());
                // System.out.println("y: " + yPID.getPositionError());
                // pivotSubsystem.setFieldPosition(swerveSubsystem.getRobotPosition());
            }, swerveSubsystem));

            driveController.getFieldResetButton().onTrue(new InstantCommand(() -> {
                swerveSubsystem.resetDriverHeading();
            }));

        } else if (baseSwerveSubsystem instanceof TestSingleModuleSwerveSubsystem) {
            final TestSingleModuleSwerveSubsystem testSwerveSubsystem = (TestSingleModuleSwerveSubsystem) baseSwerveSubsystem;
            driveController.getLeftBumper().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.decrementTest();
                System.out.println(testSwerveSubsystem.getTest());
            }));

            driveController.getRightBumper().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.incrementTest();
                System.out.println(testSwerveSubsystem.getTest());
            }));

            driveController.getFieldResetButton().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.toggletoRun();
                System.out.println(testSwerveSubsystem.getRunning() ? "Running" : "Not running");
            }));

        } else if (baseSwerveSubsystem instanceof SingleModuleSwerveSubsystem) {
            final SingleModuleSwerveSubsystem swerveSubsystem = (SingleModuleSwerveSubsystem) baseSwerveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                swerveSubsystem.setDrivePowers(driveController.getForwardPower(), driveController.getLeftPower());// , 1
                                                                                                                  // *
                                                                                                                  // (controller.getRightTriggerAxis()
                                                                                                                  // -
                                                                                                                  // controller.getLeftTriggerAxis()));
            }, swerveSubsystem));

            driveController.getFieldResetButton().onTrue(new InstantCommand(() -> {
                swerveSubsystem.toggletoRun();
            }));};

        }
        

    public Command getAutonomousCommand() {
        if (baseSwerveSubsystem instanceof SwerveSubsystem) {
            final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;
            PIDController thetacontroller = new PIDController(4, 0, 0); // TODO: tune
            thetacontroller.enableContinuousInput(-Math.PI, Math.PI);

            swerveSubsystem.resetPose(traj.getInitialPose());

            Command swerveCommand = Choreo.choreoSwerveCommand(
                    traj,
                    swerveSubsystem::getRobotPosition,
                    xPID, // X TODO: tune
                    yPID, // Y TODO: tune
                    thetacontroller,
                    ((ChassisSpeeds speeds) -> {
                        swerveSubsystem.setChassisSpeeds(
                                speeds.vxMetersPerSecond,
                                speeds.vyMetersPerSecond,
                                speeds.omegaRadiansPerSecond);
                        System.out.println(speeds.vxMetersPerSecond);
                    }),
                    isRed,
                    swerveSubsystem);

            return Commands.sequence(
                    // ahrs not resetting on own
                    // Commands.runOnce(() -> swerveSubsystem.resetAhrs()),
                    Commands.runOnce(() -> swerveSubsystem.resetPose(traj.getInitialPose())),
                    swerveCommand);
        } else
            return null;
    }

}