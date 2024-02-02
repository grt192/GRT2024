// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimbIdleCommand;
import frc.robot.commands.ClimbLowerCommand;
import frc.robot.commands.ClimbRaiseCommand;
import frc.robot.subsystems.TestMotorSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.ClimbConstants;

public class RobotContainer {
  // private final BaseSwerveSubsystem baseSwerveSubsystem;
  private final boolean IS_MANUAL = false;
  
  private final TestMotorSubsystem testClimbLeft;
  private final TestMotorSubsystem testClimbRight;
  private final ClimbSubsystem climbSubsystem;


      
  private final XboxController controller = new XboxController(0);
  // private final SwerveModule module;

  // private final JoystickButton
    // LBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value),
    // RBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value),
    // AButton = new JoystickButton(controller, XboxController.Button.kA.value);

  private final JoystickButton
    YButton = new JoystickButton(controller, XboxController.Button.kY.value),
    AButton = new JoystickButton(controller, XboxController.Button.kA.value),
    BButton = new JoystickButton(controller, XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //construct Test
    // module = new SwerveModule(0, 1, 0);
    // baseSwerveSubsystem = new SingleModuleSwerveSubsystem(module);
    // baseSwerveSubsystem = new SwerveSubsystem();
    if (IS_MANUAL) {
      climbSubsystem = new ClimbSubsystem();
      testClimbLeft = null;
      testClimbRight = null;
    } else {
      climbSubsystem = null;
      testClimbLeft = new TestMotorSubsystem(ClimbConstants.LEFT_WINCH_MOTOR_ID, true);
      testClimbRight = new TestMotorSubsystem(ClimbConstants.RIGHT_WINCH_MOTOR_ID, false);
    }
    // Configure the trigger bindings
    configureBindings();    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // if(baseSwerveSubsystem instanceof SwerveSubsystem){
    //   final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;

    //   swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
    //     swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY(), -controller.getRightX());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    //   }
    //   , swerveSubsystem));

    //   AButton.onTrue(new InstantCommand(() -> {
    //     swerveSubsystem.resetDriverHeading();
    //   }
    //   ));
      
    // } else if(baseSwerveSubsystem instanceof TestSingleModuleSwerveSubsystem){
    //   final TestSingleModuleSwerveSubsystem testSwerveSubsystem = (TestSingleModuleSwerveSubsystem) baseSwerveSubsystem;
    //   LBumper.onTrue(new InstantCommand(() -> {
    //     testSwerveSubsystem.decrementTest();
    //     System.out.println(testSwerveSubsystem.getTest());
    //   }
    //   ));

    //   RBumper.onTrue(new InstantCommand(() -> {
    //     testSwerveSubsystem.incrementTest();
    //     System.out.println(testSwerveSubsystem.getTest());
    //   }
    //   ));

    //   AButton.onTrue(new InstantCommand(() -> {
    //     testSwerveSubsystem.toggletoRun();
    //     System.out.println(testSwerveSubsystem.getRunning() ? "Running" : "Not running");
    //   }));

    // } else if (baseSwerveSubsystem instanceof SingleModuleSwerveSubsystem){
    //   final SingleModuleSwerveSubsystem swerveSubsystem = (SingleModuleSwerveSubsystem) baseSwerveSubsystem;

    //   System.out.println("1");

    //   swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
    //     swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    //   }
    //   , swerveSubsystem));

    //   AButton.onTrue(new InstantCommand(() -> {
    //     swerveSubsystem.toggletoRun();
    //   }));
    // }

    /* CLIMB */
    if (IS_MANUAL) {
      testClimbLeft.setDefaultCommand(new RunCommand(() -> {
        testClimbLeft.setMotorSpeed(controller.getLeftTriggerAxis() * (controller.getBButton() ? -1 : +1));
      }, testClimbLeft));

      testClimbRight.setDefaultCommand(new RunCommand(() -> {
        testClimbRight.setMotorSpeed(controller.getRightTriggerAxis() * (controller.getBButton() ? -1 : +1));
      }, testClimbRight));
    } else {
      YButton.onTrue(new ClimbLowerCommand(climbSubsystem));
      AButton.onTrue(new ClimbRaiseCommand(climbSubsystem));
      BButton.onTrue(new ClimbIdleCommand(climbSubsystem));
    }
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
