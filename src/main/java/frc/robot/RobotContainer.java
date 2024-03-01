// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
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
import frc.robot.commands.auton.AutonFactoryFunction;
import frc.robot.commands.auton.Bottom2PieceSequence;
import frc.robot.commands.auton.BottomPreloadedSequence;
import frc.robot.commands.auton.Middle2PieceSequence;
import frc.robot.commands.auton.Middle3PieceSequence;
import frc.robot.commands.auton.Middle4PieceSequence;
import frc.robot.commands.auton.MiddlePreloadedSequence;
import frc.robot.commands.auton.Top2PieceSequence;
import frc.robot.commands.auton.TopPreloadedSequence;
import frc.robot.commands.auton.speakertomiddletoamp;
import frc.robot.commands.climb.ClimbLowerCommand;
import frc.robot.commands.climb.ClimbRaiseCommand;
import frc.robot.commands.elevator.ElevatorSetManualCommand;
import frc.robot.commands.elevator.ElevatorToAMPCommand;
import frc.robot.commands.elevator.ElevatorToIntakeCommand;
import frc.robot.commands.elevator.ElevatorToTrapCommand;
import frc.robot.commands.elevator.ElevatorToZeroCommand;
import frc.robot.commands.intake.pivot.IntakePivotMiddleCommand;
import frc.robot.commands.intake.roller.IntakeRollerFeedCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.commands.sequences.AutoIntakeSequence;
import frc.robot.commands.sequences.ShootModeSequence;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelReadyCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotSetAngleCommand;
import frc.robot.commands.shooter.pivot.ShooterPivotVerticalCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.NoteAlignCommand;
import frc.robot.commands.swerve.SwerveStopCommand;
import frc.robot.subsystems.climb.ManualClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;
import frc.robot.util.ConditionalWaitCommand;
import frc.robot.util.GRTUtil;
import frc.robot.vision.NoteDetectionWrapper;

import static frc.robot.Constants.VisionConstants.NOTE_CAMERA;

import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.util.PixelFormat;
import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

/** The robot container. */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final BaseDriveController driveController;
    private final BaseSwerveSubsystem baseSwerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollersSubsystem intakeRollerSubsystem = new IntakeRollersSubsystem();

    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;

    private final ManualClimbSubsystem climbSubsystem;
    // private final ClimbSubsystem climbSubsystem;

    private final ElevatorSubsystem elevatorSubsystem;

    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    private final NoteDetectionWrapper noteDetector;

    private final SendableChooser<AutonFactoryFunction> autonPathChooser;

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
    private final JoystickButton offsetUpButton = new JoystickButton(switchboard, 7);
    private final JoystickButton offsetDownButton = new JoystickButton(switchboard, 8);

    private UsbCamera camera1;
    private MjpegServer mjpegServer1;

    ChoreoTrajectory trajectory;
    // private final SwerveModule module;

    private PIDController xPID;
    private PIDController yPID;

    private final GenericEntry xError;
    private final GenericEntry yError;

    private final ShuffleboardTab swerveCrauton;

    private final BooleanSupplier isRed;

    private double shooterPivotSetPosition = Units.degreesToRadians(18);
    private double shooterTopSpeed = .1;
    private double shooterBotSpeed = .1;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // construct Test
        // module = new SwerveModule(20, 1, 0);
        // baseSwerveSubsystem = new TestSingleModuleSwerveSubsystem(module);
        isRed = () -> false; // DriverStation.getAlliance() == new Optional<Alliance> ;
        baseSwerveSubsystem = new SwerveSubsystem();
        intakePivotSubsystem = new IntakePivotSubsystem();

        shooterPivotSubsystem = new ShooterPivotSubsystem(baseSwerveSubsystem::getRobotPosition);
        shooterFlywheelSubsystem = new ShooterFlywheelSubsystem(baseSwerveSubsystem::getRobotPosition);

        // climbSubsystem = new ClimbSubsystem();

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
        // private final SwerveModule module;
        // construct Test
        // module = new SwerveModule(6, 7, 0);
        // baseSwerveSubsystem = new TestSingleModuleSwerveSubsystem(module);
        // baseSwerveSubsystem = new SwerveSubsystem();
        // intakePivotSubsystem = new IntakePivotSubsystem();

        noteDetector = new NoteDetectionWrapper(NOTE_CAMERA);

        UsbCamera camera1 = new UsbCamera("fisheye", 0);
        // UsbCamera camera = CameraServer.startAutomaticCapture(0);
        // camera.setExposureManual(30);
        // camera.setFPS(60);
        // camera.setBrightness(45);
        // camera1 = new UsbCamera("camera1", 0);
        
        // camera1.setFPS(60);
        // camera1.setBrightness(3);
        // camera1.setResolution(320, 240);
        // camera1.setVideoMode()
        camera1.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
        mjpegServer1 = new MjpegServer("m1", 1181);
        mjpegServer1.setSource(camera1);

        autonPathChooser = new SendableChooser<>();
        autonPathChooser.setDefaultOption("TOPPreloaded", TopPreloadedSequence::new); 
        autonPathChooser.addOption("TOP2Piece", Top2PieceSequence::new); 
        autonPathChooser.addOption("MIDDLEShootPreloaded", MiddlePreloadedSequence::new); 
        autonPathChooser.addOption("MIDDLE2Piece", Middle2PieceSequence::new); 
        autonPathChooser.addOption("BOTTOMShootPreloaded", BottomPreloadedSequence::new); 
        autonPathChooser.addOption("BOTTOM2Piece", Bottom2PieceSequence::new); 

        // Configure the trigger bindings
        configureBindings();

    }

    private void configureBindings() {


        // SHOOTER PIVOT TEST

        // rightBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem, Units.degreesToRadians(18)));

        // leftBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem, Units.degreesToRadians(60)));

        // SHOOTER PIVOT TUNE

        shooterPivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
            // shooterPivotSubsystem.setAngle(shooterPivotSetPosition);
            if (mechController.getPOV() == 0) {
                shooterPivotSetPosition += .003;
            } else if (mechController.getPOV() == 180) {
                shooterPivotSetPosition -= .003;
            } else if (mechController.getPOV() == 45) {
                shooterTopSpeed += .001;
            } else if (mechController.getPOV() == 315) {
                shooterTopSpeed -= .001;
            } else if (mechController.getPOV() == 135) {
                shooterBotSpeed += .001;
            } else if (mechController.getPOV() == 225) {
                shooterBotSpeed -= .001;
            }
            // System.out.print(" Top: " + GRTUtil.twoDecimals(shooterTopSpeed) + " Bot: " + GRTUtil.twoDecimals(shooterBotSpeed));

            // shooterPivotSubsystem.getAutoAimAngle();


        }, shooterPivotSubsystem
        ));

        //ElEVATOR TEST

        // rightBumper.onTrue(new ElevatorToAMPCommand(elevatorSubsystem));

        // leftBumper.onTrue(new ElevatorToZeroCommand(elevatorSubsystem));


        //INTAKE TEST

        // xButton.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(.3), intakePivotSubsystem));

        // rightBumper.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem));

        // leftBumper.onTrue(new InstantCommand(() -> intakePivotSubsystem.setPosition(.85), intakePivotSubsystem));


        //NORMAL BINDS

        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            climbSubsystem.setSpeeds(-mechController.getLeftY(), -mechController.getRightY());
        }, climbSubsystem));

        // TOGGLES THE ELEVATOR FOR AMP
        rightBumper.onTrue(
            new ConditionalCommand(
                new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(
                () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem
            )), 
            new IntakePivotMiddleCommand(intakePivotSubsystem, 1).andThen(
                new IntakeRollerOuttakeCommand(intakeRollerSubsystem).until(() -> intakeRollerSubsystem.getFrontSensor() > .12),
                new ElevatorToAMPCommand(elevatorSubsystem),
                new IntakePivotMiddleCommand(intakePivotSubsystem, 0)
            ), 
            () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP || elevatorSubsystem.getTargetState() == ElevatorState.TRAP)
        );
            

        leftBumper.onTrue(
            new ConditionalCommand(
                new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(
                    () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem
                )), 
                    new ElevatorToTrapCommand(elevatorSubsystem), 
            () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP || elevatorSubsystem.getTargetState() == ElevatorState.TRAP)
        );

        
            
        // GRTUtil.getBinaryCommandChoice(

        //     () -> elevatorSubsystem.getTargetState() == ElevatorState.TRAP,
        //     new ElevatorToTrapCommand(elevatorSubsystem),
        //     new ElevatorToZeroCommand(elevatorSubsystem)));

        aButton.onTrue(
            new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
                new IntakePivotMiddleCommand(intakePivotSubsystem, 1).alongWith(
                    new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem)).andThen(
                        new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow)
                        // new IntakePivotMiddleCommand(intakePivotSubsystem, 0) // TODO: ADD THIS 
                    ).unless(() -> mechController.getLeftTriggerAxis() > .1) //CANCEL IF TRY TO OUTTAKE
                ).until(intakeRollerSubsystem::backSensorNow)


            // GRTUtil.getBinaryCommandChoice(intakeRollerSubsystem::frontSensorNow, 
            //     new ElevatorToIntakeCommand(elevatorSubsystem).andThen(
            //         new IntakePivotMiddleCommand(intakePivotSubsystem, 1).alongWith(
            //             new IntakeRollerIntakeCommand(intakeRollerSubsystem, ledSubsystem)).andThen(
            //                 new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow)
            //                 // new IntakePivotMiddleCommand(intakePivotSubsystem, 0) // TODO: ADD THIS 
            //             ).unless(() -> mechController.getLeftTriggerAxis() > .1) //CANCEL IF TRY TO OUTTAKE
            //         ).until(intakeRollerSubsystem::backSensorNow),
            //     new IntakePivotMiddleCommand(intakePivotSubsystem, 1).andThen(
            //         new IntakeRollerFeedCommand(intakeRollerSubsystem).until(intakeRollerSubsystem::backSensorNow)
            //         // new IntakeRollerFeedCommand(intakeRollerSubsystem).withTimeout(.1),
            //         // new IntakePivotMiddleCommand(intakePivotSubsystem, 0) // TODO: ADD THIS 
            //     )
            // ).unless(intakeRollerSubsystem::backSensorNow)
        );

        bButton.onTrue(new InstantCommand(() -> {}, intakeRollerSubsystem)
        );

        shooterFlywheelSubsystem.setDefaultCommand(new InstantCommand(() -> {
            if (yButton.getAsBoolean()) {
                shooterFlywheelSubsystem.setShooterMotorSpeed(shooterTopSpeed, shooterBotSpeed);
            } else {
                shooterFlywheelSubsystem.setShooterMotorSpeed(0);
            }
        }, shooterFlywheelSubsystem

        ));

        // yButton.onTrue(new ShooterFlywheelReadyCommand(shooterFlywheelSubsystem).alongWith(
        //     // new InstantCommand(() -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)
        // ));

        yButton.onFalse(new ShooterFlywheelStopCommand(shooterFlywheelSubsystem));



        intakeRollerSubsystem.setDefaultCommand(new InstantCommand(() -> {
            double power = 0; 
            
            if (intakeRollerSubsystem.backSensorNow()) {
                if (shooterFlywheelSubsystem.atSpeed()) {
                    power = mechController.getRightTriggerAxis() > .1 ? 1 : .7 * (- mechController.getLeftTriggerAxis());
                } else {
                    power = .7 * (- mechController.getLeftTriggerAxis());
                }

            } else {
                power =  .7 * (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis()); 
            }
            intakeRollerSubsystem.setAllRollSpeed(power, power);
        }, intakeRollerSubsystem));

        xButton.onTrue(new InstantCommand(() ->  intakePivotSubsystem.setPosition(1), intakePivotSubsystem));

        offsetUpButton.onTrue(new InstantCommand(() -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(5))));
        offsetUpButton.onFalse(new InstantCommand(() -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0))));

        offsetDownButton.onTrue(new InstantCommand(() -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(-5))));
        offsetDownButton.onFalse(new InstantCommand(() -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0))));

        if (baseSwerveSubsystem instanceof SwerveSubsystem) {

            driveController.getTurnModeButton().onTrue(new InstantCommand(() -> shooterPivotSubsystem.setAutoAimBoolean(true), shooterPivotSubsystem ));
            
            driveController.getTurnModeButton().onFalse(new InstantCommand(() -> shooterPivotSubsystem.setAutoAimBoolean(false), shooterPivotSubsystem ));

            final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;
            swerveCrauton.add("AUTO ALIGN BLUE AMP",
                    AlignCommand.getAlignCommand(AutoAlignConstants.BLUE_AMP_POSE, swerveSubsystem));

            ledSubsystem.setDefaultCommand(new RunCommand(() -> {
                ledSubsystem.setDriverHeading(
                    new Rotation2d(
                        driveController.getRelativeMode() ? 0 : -swerveSubsystem.getDriverHeading().getRadians()
                    )
                );
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
                    if (driveController.getSwerveAimMode()) {
                        swerveSubsystem.setSwerveAimDrivePowers(
                            driveController.getForwardPower(), 
                            driveController.getLeftPower()
                        );
                    } else {
                        swerveSubsystem.setDrivePowers(
                            driveController.getForwardPower(), 
                            driveController.getLeftPower(), 
                            driveController.getRotatePower()
                        );
                    }
                }
                // pivotSubsystem.setFieldPosition(swerveSubsystem.getRobotPosition());
                xError.setValue(xPID.getPositionError());
                yError.setValue(yPID.getPositionError());
                // System.out.println("y: " + yPID.getPositionError());
                // pivotSubsystem.setFieldPosition(swerveSubsystem.getRobotPosition());
            }, swerveSubsystem));

            driveController.getDriverHeadingResetButton().onTrue(new InstantCommand(() -> {
                swerveSubsystem.resetDriverHeading();
            }));

        } else if (baseSwerveSubsystem instanceof TestSingleModuleSwerveSubsystem) {
            final TestSingleModuleSwerveSubsystem testSwerveSubsystem = 
                (TestSingleModuleSwerveSubsystem) baseSwerveSubsystem;
            driveController.getLeftBumper().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.decrementTest();
                System.out.println(testSwerveSubsystem.getTest());
            }));

            driveController.getRightBumper().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.incrementTest();
                System.out.println(testSwerveSubsystem.getTest());
            }));

            driveController.getDriverHeadingResetButton().onTrue(new InstantCommand(() -> {
                testSwerveSubsystem.toggleToRun();
                System.out.println(testSwerveSubsystem.getRunning() ? "Running" : "Not running");
            }));

        } else if (baseSwerveSubsystem instanceof SingleModuleSwerveSubsystem) {
            final SingleModuleSwerveSubsystem swerveSubsystem = (SingleModuleSwerveSubsystem) baseSwerveSubsystem;

            swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
                swerveSubsystem.setDrivePowers(driveController.getForwardPower(), driveController.getLeftPower());
            }, swerveSubsystem));

            driveController.getDriverHeadingResetButton().onTrue(new InstantCommand(() -> {
                swerveSubsystem.toggleToRun();
            }));};

        }
        

    /** Returns the autonomous command.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        if (!(baseSwerveSubsystem instanceof SwerveSubsystem)) return null;
        
        return new Middle4PieceSequence(intakePivotSubsystem, intakeRollerSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, (SwerveSubsystem) baseSwerveSubsystem, ledSubsystem);//autonPathChooser.getSelected().create(intakePivotSubsystem, intakeRollerSubsystem, shooterFlywheelSubsystem, shooterPivotSubsystem, elevatorSubsystem, (SwerveSubsystem) baseSwerveSubsystem, ledSubsystem);
    }

}
