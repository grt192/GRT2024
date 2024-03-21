// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.VisionConstants.NOTE_CAMERA;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton.AutonBuilder;
import frc.robot.commands.climb.ClimbLowerCommand;
import frc.robot.commands.climb.ClimbRaiseCommand;
import frc.robot.commands.elevator.ElevatorToAmpCommand;
import frc.robot.commands.elevator.ElevatorToTrapCommand;
import frc.robot.commands.elevator.ElevatorToZeroCommand;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelShuttleCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.commands.swerve.SwerveStopCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;
import frc.robot.vision.NoteDetectionWrapper;

/** The robot container. */
public class RobotContainer {
    private final BaseDriveController driveController;
    private final SwerveSubsystem swerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;

    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;

    private final ClimbSubsystem climbSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final FieldManagementSubsystem fmsSubsystem;
    private final LightBarSubsystem lightBarSubsystem;
    private final SuperstructureSubsystem superstructureSubsystem;

    private final SendableChooser<Command> autonPathChooser;
    private final AutonBuilder autonBuilder;

    private final NoteDetectionWrapper noteDetector;

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
    private final JoystickButton leftStickButton = new JoystickButton(mechController,
            XboxController.Button.kLeftStick.value);
    private final JoystickButton rightStickButton = new JoystickButton(mechController,
        XboxController.Button.kRightStick.value);

    private final GenericHID switchboard = new GenericHID(3);
    private final JoystickButton offsetUpButton = new JoystickButton(switchboard, 7);
    private final JoystickButton offsetDownButton = new JoystickButton(switchboard, 8);

    private final JoystickButton toggleClimbModeSwitch = new JoystickButton(switchboard, 6);
    private final JoystickButton shuttleNotes = new JoystickButton(switchboard, 6);

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
    private double intakePosition = 0;

    private boolean isNoteTrapReady = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        fmsSubsystem = new FieldManagementSubsystem();
        lightBarSubsystem = new LightBarSubsystem();
        superstructureSubsystem = new SuperstructureSubsystem(lightBarSubsystem, fmsSubsystem);
        
        swerveSubsystem = new SwerveSubsystem(fmsSubsystem::isRedAlliance);
        swerveSubsystem.setVerbose(false); // SET THIS TO true FOR TUNING VALUES

        intakePivotSubsystem = new IntakePivotSubsystem();
        intakeRollerSubsystem = new IntakeRollerSubsystem(lightBarSubsystem);

        shooterPivotSubsystem = new ShooterPivotSubsystem(
            swerveSubsystem::getRobotPosition, 
            fmsSubsystem::isRedAlliance
        );
        shooterFlywheelSubsystem = new ShooterFlywheelSubsystem(
            swerveSubsystem::getRobotPosition, 
            fmsSubsystem::isRedAlliance
        );

        elevatorSubsystem = new ElevatorSubsystem();

        climbSubsystem = new ClimbSubsystem();
        climbSubsystem.setManual();

        noteDetector = new NoteDetectionWrapper(NOTE_CAMERA);

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
        driverCamera.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
        driverCamera.setExposureManual(35);
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
        autonPathChooser.addOption("topPreloaded", autonBuilder.getMiddleFourPiece());
        autonPathChooser.addOption("top2Piece", autonBuilder.getTopTwoPiece());
        autonPathChooser.addOption("top3Piece", autonBuilder.getTopThreePiece());
        autonPathChooser.addOption("top4Piece", autonBuilder.getTopFourPiece());
        // autonPathChooser.addOption("centerTop2Piece", autonBuilder.());
        autonPathChooser.setDefaultOption("middlePreloaded", autonBuilder.getMiddlePreloaded());
        autonPathChooser.addOption("middle2Piece", autonBuilder.getMiddleTwoPiece());
        autonPathChooser.addOption("middle3Piece", autonBuilder.getMiddleThreePiece());
        autonPathChooser.addOption("middle2PieceThenCenterTop1", autonBuilder.get2TopWingThen1Center1()); //UNTESTED
        autonPathChooser.addOption("middle2PieceThenCenterTop2", autonBuilder.get2TopWingThen2TopCenter()); //UNTESTED
        autonPathChooser.addOption("middle4Piece", autonBuilder.getMiddleFourPiece());
        autonPathChooser.addOption("bottomPreloaded", autonBuilder.getBottomPreloaded());
        autonPathChooser.addOption("bottom2Piece", autonBuilder.getBottomTwoPiece());

        swerveCrauton.add(autonPathChooser);

        configureBindings();
    }

    private void configureBindings() {
        
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
                
                case 90:
                    intakePosition += .01;
                    break;
                
                case 270:
                    intakePosition -= .01;
                    break;

                default:
                    break;
            }

            // System.out.println(intakePosition);


            
            // System.out.print(" Top: " + GRTUtil.twoDecimals(shooterTopSpeed)
            //                + " Bot: " + GRTUtil.twoDecimals(shooterBotSpeed)
            // );

            // shooterPivotSubsystem.getAutoAimAngle();
            // shooterPivotSubsystem.setAngle(shooterPivotSetPosition);
        }, shooterPivotSubsystem));

        /* ElEVATOR TEST */

        // rightBumper.onTrue(new ElevatorToAMPCommand(elevatorSubsystem));
        // leftBumper.onTrue(new ElevatorToZeroCommand(elevatorSubsystem));

        // elevatorSubsystem.setManual();

        // elevatorSubsystem.setDefaultCommand(new InstantCommand(() ->
        //     elevatorSubsystem.setManualPower(mechController.getRightX()),
        //     elevatorSubsystem)
        // );

        /* INTAKE TEST */

        // xButton.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(.3),
        // intakePivotSubsystem));

        // rightBumper.onTrue(new InstantCommand(() ->
        // intakePivotSubsystem.setPosition(0), intakePivotSubsystem));

        // leftBumper.onTrue(new InstantCommand(() ->
        // intakePivotSubsystem.setPosition(.85), intakePivotSubsystem));

        /* MECHANISM BINDINGS */

        /* Climb Controls -- In manual mode, left and right joystick up/down controls left and right arm up/down,
         * respectively. In automatic mode, pressing the left and right joysticks sends the elevator to its lowered
         * and raised positions.*/
        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            climbSubsystem.setSpeeds(-mechController.getLeftY(), -mechController.getRightY());
        }, climbSubsystem));

        leftStickButton.onTrue(new ClimbLowerCommand(climbSubsystem));
        rightStickButton.onTrue(new ClimbRaiseCommand(climbSubsystem));

        toggleClimbModeSwitch.onTrue(new InstantCommand(() -> climbSubsystem.setManual()));
        toggleClimbModeSwitch.onFalse(new InstantCommand(() -> climbSubsystem.setAutomatic()));

        // rightBumper toggles the amp sequence 
        // if the elevator is up, lower it and stow the intake
        // if the elevator is down, run the amp sequence
        rightBumper.onTrue(
            new ConditionalCommand(
                    // if elevator is up
                    new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(// lower the elevator
                        () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)), // stow the pivot
                    // if elevator is down
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).andThen(// extend pivot
                        new IntakeRollerOuttakeCommand(intakeRollerSubsystem, .17, .75) // run rollers to front sensor
                                .until(() -> intakeRollerSubsystem.getFrontSensorReached()),
                        new ElevatorToAmpCommand(elevatorSubsystem), // raise elevator
                        new IntakePivotSetPositionCommand(intakePivotSubsystem, 0.2) // angle intake for scoring
                    ).until(() -> mechController.getLeftTriggerAxis() > .05 
                        || mechController.getRightTriggerAxis() > .05
                    ), 
                    // check if the elevator is currently targeting one of the upper positions to choose what to do
                    () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP
                        || elevatorSubsystem.getTargetState() == ElevatorState.TRAP));

        // leftBumper toggles the trap position for the elevator
        leftBumper.onTrue(
            new ConditionalCommand(
                new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(// lower the elevator
                    () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)), // stow intake
                new ConditionalCommand(
                    new ElevatorToTrapCommand(elevatorSubsystem).andThen(
                        new IntakePivotSetPositionCommand(intakePivotSubsystem, .45).withTimeout(.1)
                    ).andThen(new InstantCommand(() -> {
                        isNoteTrapReady = false;
                    })), 
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).andThen(// extend pivot
                        new IntakeRollerOuttakeCommand(intakeRollerSubsystem, .17, .75) // run rollers to front sensor
                                .until(() -> intakeRollerSubsystem.getFrontSensorReached()),
                        new IntakePivotSetPositionCommand(intakePivotSubsystem, 0)
                    ).andThen(new InstantCommand(() -> {
                        isNoteTrapReady = true;
                    })),
                    () -> isNoteTrapReady
                ), // raise the elevator
                () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP // check if targeting a high pos
                    || elevatorSubsystem.getTargetState() == ElevatorState.TRAP)
        );



        // aButton runs the intake sequence
        aButton.onTrue(
            new ElevatorToZeroCommand(elevatorSubsystem).andThen(// first lower the elevator (should be down)
                new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).alongWith(// then extend the intake
                new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)).andThen(
                    // intake the note to the color sensor
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 0) // stow intake
                ).unless(() -> mechController.getLeftTriggerAxis() > .1) // cancel if try to outtake
            )
        );

        // bButton stops the rollers
        bButton.onTrue(new InstantCommand(() -> {}, intakeRollerSubsystem));

        // xButton toggles the intake being stowed
        xButton.onTrue(new InstantCommand(() ->  {
            double outPosition = 1;
            double stowPosition = 0;
            if (elevatorSubsystem.getExtensionPercent() > .5 
                && elevatorSubsystem.getTargetState() == ElevatorState.TRAP) {
                outPosition = .45; // push intake out
            } else if (elevatorSubsystem.getExtensionPercent() > .5 
                && elevatorSubsystem.getTargetState() == ElevatorState.AMP) {
                stowPosition = .2;
            }

            intakePivotSubsystem.setPosition(
                intakePivotSubsystem.getEncoderPosition() < (outPosition + stowPosition) / 2 
                ? outPosition 
                : stowPosition
            );

        }, intakePivotSubsystem));

        // yButton runs the flywheels
        shooterFlywheelSubsystem.setDefaultCommand(new InstantCommand(() -> {
            if (yButton.getAsBoolean()) {
                lightBarSubsystem.setLightBarStatus(LightBarStatus.SHOOTER_SPIN_UP, 2);
                // shooterFlywheelSubsystem.setShooterMotorSpeed(shooterTopSpeed, shooterBotSpeed); // for tuning
                shooterFlywheelSubsystem.setShooterMotorSpeed();
                shooterPivotSubsystem.setAutoAimBoolean(true);
            } else {
                shooterPivotSubsystem.setAutoAimBoolean(false);
                shooterFlywheelSubsystem.stopShooter();
            }

            // if we are at speed, rumble the mech controller
            if (shooterFlywheelSubsystem.atSpeed()) {
                mechController.setRumble(RumbleType.kBothRumble, .4);
            } else {
                mechController.setRumble(RumbleType.kBothRumble, 0);
                if (lightBarSubsystem.getLightBarMechStatus() == LightBarStatus.SHOOTER_SPIN_UP) {
                    double top = shooterFlywheelSubsystem.getTopSpeed() / shooterFlywheelSubsystem.getTargetTopRPS();
                    double bottom = shooterFlywheelSubsystem.getBottomSpeed() 
                                  / shooterFlywheelSubsystem.getTargetBottomRPS();
                    double avg = (top + bottom) / 2; // in case they're different, this just shows the average. 

                    lightBarSubsystem.updateShooterSpeedPercentage(avg);
                }
                
            }
        }, shooterFlywheelSubsystem
        ));

        // intakePivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
        //     intakePosition = MathUtil.clamp(intakePosition, 0, 1);
        //     intakePivotSubsystem.setPosition(intakePosition);
        // }, intakePivotSubsystem));

        // The triggers intake/outtake the rollers
        intakeRollerSubsystem.setDefaultCommand(new InstantCommand(() -> {

            double power = .7 * (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis());

            intakeRollerSubsystem.setRollSpeeds(power, power);
        }, intakeRollerSubsystem));

        // Offset buttons to correct the shooter if needed
        offsetUpButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(3)))
        );
        offsetUpButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );

        offsetDownButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(-3)))
        );
        offsetDownButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );

        shuttleNotes.onTrue(new ShooterFlywheelShuttleCommand(swerveSubsystem, 
            shooterFlywheelSubsystem, swerveSubsystem::getRobotPosition, shooterPivotSubsystem)
        );

        /* SWERVE BINDINGS */

        /* Shooter Aim -- Holding down the button will change the shooter's pitch to aim it at the speaker. */
        // drive

        /* Amp Align -- Pressing and holding the button will cause the robot to automatically path find to the amp.
         * Releasing the button will stop the robot (and the path finding). */
        driveController.getAmpAlign().onTrue(new InstantCommand(
            () -> lightBarSubsystem.setLightBarStatus(LightBarStatus.AUTO_ALIGN, 1)
            ).andThen(new ParallelRaceGroup(
                AlignCommand.getAmpAlignCommand(swerveSubsystem, fmsSubsystem.isRedAlliance()),
                new ConditionalWaitCommand(
                    () -> !driveController.getAmpAlign().getAsBoolean()))
                    ).andThen(new InstantCommand(() -> lightBarSubsystem.setLightBarStatus(LightBarStatus.DORMANT, 1)))
        );

        /* Note align -- deprecated, new version in the works*/
        driveController.getNoteAlign().onTrue(
            new NoteAlignCommand(swerveSubsystem, noteDetector, driveController)
                .unless(() -> noteDetector.getNote().isEmpty())
        );
    
        /* Swerve Stop -- Pressing the button completely stops the robot's motion. */
        driveController.getSwerveStop().onTrue(new SwerveStopCommand(swerveSubsystem));

        
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