// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;
import java.util.function.BooleanSupplier;
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    private final IntakeSubsystem intakeSubsystem  = new IntakeSubsystem();
    private final BaseDriveController driveController = new DualJoystickDriveController();
    private final BaseSwerveSubsystem baseSwerveSubsystem;

    private final IntakePivotSubsystem intakePivotSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;


    private final XboxController mechController = new XboxController(2);
    private final JoystickButton aButton = new JoystickButton(mechController, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(mechController, XboxController.Button.kB.value);
    private final JoystickButton leftBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);
    private final JoystickButton xButton = new JoystickButton(mechController, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(mechController, XboxController.Button.kY.value);
    //private final JoystickButton xButton = new JoystickButton(mechController, XboxController.Button.kX.value);

    ChoreoTrajectory traj;
    // private final SwerveModule module;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //construct Test
        // module = new SwerveModule(6, 7, 0);
        // baseSwerveSubsystem = new TestSingleModuleSwerveSubsystem(module);
      baseSwerveSubsystem = new SwerveSubsystem();
      intakePivotSubsystem = new IntakePivotSubsystem();
      feederSubsystem = new FeederSubsystem();

      shooterPivotSubsystem = new ShooterPivotSubsystem(false);
      shooterSubsystem = new ShooterSubsystem();
      
      traj = Choreo.getTrajectory("Curve");

    // Configure the trigger bindings
    configureBindings();
  }


    private void configureBindings() {

        /**
         * Right trigger - run pivot forward at speed pulled
         * Left trigger - run pivot backward at speed pulled
         * A button - run flywheels while button held
         * B button - run feed wheels while button held
         */

        aButton.onTrue(new LoadNoteCommand(feederSubsystem));

        bButton.onTrue(new ShootNoteCommand(feederSubsystem));

        leftBumper.onTrue(new AutoAimCommand(pivotSubsystem));

        rightBumper.onTrue(new VerticalCommand(pivotSubsystem));

        xButton.onTrue(new ReadyShooterCommand(shooterSubsystem));

        yButton.onTrue(new StopShooterCommands(shooterSubsystem));

        intakePivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
            intakePivotSubsystem.setPivotSpeed(-controller.getLeftTriggerAxis());
        }));

        shooterSubsystem.setDefaultCommand(new InstantCommand(() -> {
            intakePivotSubsystem.setPivotSpeed(controller.getRightTriggerAxis());
        }));



        if(baseSwerveSubsystem instanceof SwerveSubsystem){
        final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;

        swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
            swerveSubsystem.setDrivePowers(driveController.getLeftPower(), driveController.getForwardPower(), driveController.getRotatePower());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
            pivotSubsystem.setFieldPosition(swerveSubsystem.getRobotPosition());
        }
        , swerveSubsystem));

        driveController.getFieldResetButton().onTrue(new InstantCommand(() -> {
            swerveSubsystem.resetDriverHeading();
        }
        ));
        
        } else if(baseSwerveSubsystem instanceof TestSingleModuleSwerveSubsystem){
        final TestSingleModuleSwerveSubsystem testSwerveSubsystem = (TestSingleModuleSwerveSubsystem) baseSwerveSubsystem;
        // LBumper.onTrue(new InstantCommand(() -> {
        //   testSwerveSubsystem.decrementTest();
        //   System.out.println(testSwerveSubsystem.getTest());
        // }
        // ));

        // RBumper.onTrue(new InstantCommand(() -> {
        //   testSwerveSubsystem.incrementTest();
        //   System.out.println(testSwerveSubsystem.getTest());
        // }
        // ));

        // AButton.onTrue(new InstantCommand(() -> {
        //   testSwerveSubsystem.toggletoRun();
        //   System.out.println(testSwerveSubsystem.getRunning() ? "Running" : "Not running");
        // }));

        } else if (baseSwerveSubsystem instanceof SingleModuleSwerveSubsystem){
        final SingleModuleSwerveSubsystem swerveSubsystem = (SingleModuleSwerveSubsystem) baseSwerveSubsystem;

        // System.out.println("1");

        // swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
        //   swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
        // }
        // , swerveSubsystem));

        // AButton.onTrue(new InstantCommand(() -> {
        //   swerveSubsystem.toggletoRun();
        // }));
        
        }

    
    } 

  public Command getAutonomousCommand() {
    if(baseSwerveSubsystem instanceof SwerveSubsystem){
      
      final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;
      PIDController thetacontroller = new PIDController(1, 0, 0); //TODO: tune
      thetacontroller.enableContinuousInput(-Math.PI, Math.PI);

      BooleanSupplier isBlue = () -> true; //DriverStation.getAlliance() == new Optional<Alliance> ; 

      Command swerveCommand = Choreo.choreoSwerveCommand(
        traj,
        swerveSubsystem::getRobotPosition, 
        new PIDController(1, 0, 0), //X TODO: tune
        new PIDController(1, 0, 0), //Y TODO: tune
        thetacontroller, 
        ((ChassisSpeeds speeds) -> swerveSubsystem.setDrivePowers(
          speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond, 
          speeds.omegaRadiansPerSecond
          )),
        isBlue,
        swerveSubsystem
        );

        return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetPose(traj.getInitialPose())),
          swerveCommand
        );
    }
    else return null;
  }
}