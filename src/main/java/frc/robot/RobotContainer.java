// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton.AutonFactoryFunction;
import frc.robot.commands.auton.Bottom2PieceSequence;
import frc.robot.commands.auton.BottomPreloadedSequence;
import frc.robot.commands.auton.Middle2PieceSequence;
import frc.robot.commands.auton.Middle4PieceSequence;
import frc.robot.commands.auton.TaxiSequence;
import frc.robot.commands.auton.Top2PieceSequence;
import frc.robot.commands.auton.TopPreloadedSequence;
import frc.robot.commands.elevator.ElevatorToAmpCommand;
import frc.robot.commands.elevator.ElevatorToIntakeCommand;
import frc.robot.commands.elevator.ElevatorToTrapCommand;
import frc.robot.commands.elevator.ElevatorToZeroCommand;
import frc.robot.commands.intake.pivot.IntakePivotMiddleCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.SwerveStopCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.climb.ManualClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;
import frc.robot.util.GRTUtil;


/** The robot container. */
public class RobotContainer {
    private final BaseDriveController driveController;
    private final SwerveSubsystem swerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollerSubsystem = new IntakeRollersSubsystem();

    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;

    private final ManualClimbSubsystem climbSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem();
    private final LightBarSubsystem lightBarSubsystem = new LightBarSubsystem();

    private final SendableChooser<AutonFactoryFunction> autonPathChooser;

    /* MECH BUTTONS */
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
    private final JoystickButton offsetUpButton = new JoystickButton(switchboard, 7);
    private final JoystickButton offsetDownButton = new JoystickButton(switchboard, 8);
    private final JoystickButton toggleClimbLimitsButton = new JoystickButton(switchboard, 9);

    private UsbCamera driverCamera;
    private MjpegServer driverCameraServer;

    ChoreoTrajectory trajectory;

    private PIDController xPID;
    private PIDController yPID;

    private final GenericEntry xError;
    private final GenericEntry yError;

    private final ShuffleboardTab swerveCrauton;

    private double shooterPivotSetPosition = Units.degreesToRadians(18);
    private double shooterTopSpeed = .75;
    private double shooterBotSpeed = .4;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem();

        intakePivotSubsystem = new IntakePivotSubsystem();

        shooterPivotSubsystem = new ShooterPivotSubsystem(swerveSubsystem::getRobotPosition);
        shooterFlywheelSubsystem = new ShooterFlywheelSubsystem(swerveSubsystem::getRobotPosition);

        elevatorSubsystem = new ElevatorSubsystem();

        climbSubsystem = new ManualClimbSubsystem();

        xPID = new PIDController(4, 0, 0);
        yPID = new PIDController(4, 0, 0);

        swerveCrauton = Shuffleboard.getTab("Auton");

        xError = swerveCrauton.add("xError", 0).withPosition(8, 0).getEntry();
        yError = swerveCrauton.add("yError", 0).withPosition(9, 0).getEntry();

        if (DriverStation.getJoystickName(0).equals("Controller (Xbox One For Windows)")) {
            driveController = new XboxDriveController();
        } else {
            driveController = new DualJoystickDriveController();
        }

        driverCamera = new UsbCamera("fisheye", 0);
        driverCamera.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
        driverCameraServer = new MjpegServer("m1", 1181);
        driverCameraServer.setSource(driverCamera);

        autonPathChooser = new SendableChooser<>();
        autonPathChooser.setDefaultOption("TOPPreloaded", TopPreloadedSequence::new);
        autonPathChooser.addOption("TOP2Piece", Top2PieceSequence::new);
        autonPathChooser.addOption("MIDDLEShootPreloaded", TaxiSequence::new);
        autonPathChooser.addOption("MIDDLE2Piece", Middle2PieceSequence::new);
        autonPathChooser.addOption("BOTTOMShootPreloaded", BottomPreloadedSequence::new);
        autonPathChooser.addOption("BOTTOM2Piece", Bottom2PieceSequence::new);

        configureBindings();
    }

    private void configureBindings() {        
        /* SHOOTER PIVOT TEST */

        // rightBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem,
        // Units.degreesToRadians(18)));

        // leftBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem,
        // Units.degreesToRadians(60)));

        /* SHOOTER PIVOT TUNE */

        shooterPivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
            //TODO: switch to enum for dpad angles
            switch (mechController.getPOV()) {
                case 0:
                    shooterPivotSetPosition += .003;
                    break;

                case 180:
                    shooterPivotSetPosition -= .003;
                    break;

                case 45:
                    shooterTopSpeed += .001;
                    break;

                case 315:
                    shooterTopSpeed -= .001;
                    break;

                case 135:
                    shooterBotSpeed += .001;
                    break;

                case 225:
                    shooterBotSpeed -= .001;
                    break;

                default:
                    break;
            }
            
            System.out.print(" Top: " + GRTUtil.twoDecimals(shooterTopSpeed)
                           + " Bot: " + GRTUtil.twoDecimals(shooterBotSpeed)
            );

            shooterPivotSubsystem.getAutoAimAngle();
        }, shooterPivotSubsystem));

        /* ElEVATOR TEST */

        // rightBumper.onTrue(new ElevatorToAMPCommand(elevatorSubsystem));
        // leftBumper.onTrue(new ElevatorToZeroCommand(elevatorSubsystem));

        /* INTAKE TEST */

        // xButton.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(.3),
        // intakePivotSubsystem));

        // rightBumper.onTrue(new InstantCommand(() ->
        // intakePivotSubsystem.setPosition(0), intakePivotSubsystem));

        // leftBumper.onTrue(new InstantCommand(() ->
        // intakePivotSubsystem.setPosition(.85), intakePivotSubsystem));

        /* MECHANISM BINDINGS */

        /* Climb Controls -- In manual mode, left and right joystick up/down controls left and right arm up/down,
         * respectively. The position limits are enabled by default, but can be disabled by toggling the bottom left
         * switch on the switchboard. */
        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            climbSubsystem.setSpeeds(-mechController.getLeftY(), -mechController.getRightY());
        }, climbSubsystem));

        toggleClimbLimitsButton.onTrue(new InstantCommand(() -> climbSubsystem.enableSoftLimits(false)));
        toggleClimbLimitsButton.onFalse(new InstantCommand(() -> climbSubsystem.enableSoftLimits(true)));

        /* Elevator controls -- */ //TODO: explain how these work
        rightBumper.onTrue(
            new ConditionalCommand(
                new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(
                    () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)),
                new IntakePivotMiddleCommand(intakePivotSubsystem, 1).andThen(
                    new IntakeRollerOuttakeCommand(intakeRollerSubsystem).until(
                        () -> intakeRollerSubsystem.getFrontSensor() > .12),
                    new ElevatorToAmpCommand(elevatorSubsystem),
                    new IntakePivotMiddleCommand(intakePivotSubsystem, 0)),
                () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP
                    || elevatorSubsystem.getTargetState() == ElevatorState.TRAP)
        );

        leftBumper.onTrue(
                new ConditionalCommand(
                        new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(
                                () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)),
                        new ElevatorToTrapCommand(elevatorSubsystem),
                        () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP
                                || elevatorSubsystem.getTargetState() == ElevatorState.TRAP));

        // GRTUtil.getBinaryCommandChoice(

        // () -> elevatorSubsystem.getTargetState() == ElevatorState.TRAP,
        // new ElevatorToTrapCommand(elevatorSubsystem),
        // new ElevatorToZeroCommand(elevatorSubsystem)));

        aButton.onTrue(
                new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                        new IntakePivotMiddleCommand(intakePivotSubsystem, 0).alongWith(
                                new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)).andThen(
                                        new IntakeRollerFeedCommand(intakeRollerSubsystem)
                                                .until(intakeRollerSubsystem::backSensorNow)
                        // new IntakePivotMiddleCommand(intakePivotSubsystem, 0) // TODO: ADD THIS
                        ).unless(() -> mechController.getLeftTriggerAxis() > .1) // CANCEL IF TRY TO OUTTAKE
                ).until(intakeRollerSubsystem::backSensorNow)
        );

        bButton.onTrue(new InstantCommand(() -> {
        }, intakeRollerSubsystem));

        shooterFlywheelSubsystem.setDefaultCommand(new InstantCommand(() -> {
            if (yButton.getAsBoolean()) {
                lightBarSubsystem.setLightBarStatus(LightBarStatus.SHOOTER_SPIN_UP);
                shooterFlywheelSubsystem.setShooterMotorSpeed(shooterTopSpeed, shooterBotSpeed);
            } else {
                shooterFlywheelSubsystem.stopShooter();
            }

            if (shooterFlywheelSubsystem.atSpeed()) {
                mechController.setRumble(RumbleType.kBothRumble, .4);
            } else {
                mechController.setRumble(RumbleType.kBothRumble, 0);
            }
        }, shooterFlywheelSubsystem

        ));

        // yButton.onTrue(new
        //     ShooterFlywheelReadyCommand(shooterFlywheelSubsystem).alongWith(
        // // new InstantCommand(() -> intakePivotSubsystem.setPosition(0),
        // // intakePivotSubsystem)
        // ));

        // yButton.onFalse(new ShooterFlywheelStopCommand(shooterFlywheelSubsystem));

        intakeRollerSubsystem.setDefaultCommand(new InstantCommand(() -> {
            double power = 0;

            // if (intakeRollerSubsystem.backSensorNow()) {
            //     if (shooterFlywheelSubsystem.atSpeed()) {
            //         power = mechController.getRightTriggerAxis() > .1 ? 1 : .7 * (-mechController.getLeftTriggerAxis());
            //     } else {
            //         power = .7 * (-mechController.getLeftTriggerAxis());
            //     }

            // } else {
                power = .7 * (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis());
            // }
            intakeRollerSubsystem.setAllRollSpeed(power, power);
        }, intakeRollerSubsystem));

        xButton.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(1), intakePivotSubsystem));

        offsetUpButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(5)))
        );
        offsetUpButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );

        offsetDownButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(-5)))
        );
        offsetDownButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );

        /* SWERVE BINDINGS */

        /* Shooter Aim -- Holding down the button will change the shooter's pitch to aim it at the speaker. */
        driveController.getShooterAimButton().onTrue(
                new InstantCommand(() -> shooterPivotSubsystem.setAutoAimBoolean(true), shooterPivotSubsystem)
        );
        driveController.getShooterAimButton().onFalse(
                new InstantCommand(() -> shooterPivotSubsystem.setAutoAimBoolean(false), shooterPivotSubsystem)
        );

        /* Amp Align -- Pressing and holding the button will cause the robot to automatically pathfind to the amp.
         * Releasing the button will stop the robot (and the pathfinding). */
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

            xError.setValue(xPID.getPositionError());
            yError.setValue(yPID.getPositionError());

        }, swerveSubsystem));

        /* Pressing the button resets the field axes to the current robot axes. */
        driveController.getDriverHeadingResetButton().onTrue(new InstantCommand(() -> {
            swerveSubsystem.resetDriverHeading();
        }));
    }

    /**
     * Returns the autonomous command.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return new Middle4PieceSequence(intakePivotSubsystem, intakeRollerSubsystem, shooterFlywheelSubsystem,
                shooterPivotSubsystem, elevatorSubsystem, (SwerveSubsystem) swerveSubsystem, lightBarSubsystem);
        // autonPathChooser.getSelected().create(intakePivotSubsystem,
        // intakeRollerSubsystem,
        // shooterFlywheelSubsystem,
        // shooterPivotSubsystem,
        // elevatorSubsystem,
        // (SwerveSubsystem)
        // baseSwerveSubsystem,
        // ledSubsystem);
    }
}