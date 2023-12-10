// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private final SingleModuleSwerveSubsystem swerveSubsystem;
      
  private final XboxController controller = new XboxController(0);
  private final SwerveModule module;

  private final JoystickButton
    LBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value),
    RBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value),
    AButton = new JoystickButton(controller, XboxController.Button.kA.value);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //construct Test
    module = new SwerveModule(2, 1, Math.toRadians(-47));
    swerveSubsystem = new SingleModuleSwerveSubsystem(module);
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
    
    if(swerveSubsystem instanceof TestSingleModuleSwerveSubsystem){
      final TestSingleModuleSwerveSubsystem testSwerveSubsystem = (TestSingleModuleSwerveSubsystem) swerveSubsystem;
      LBumper.onTrue(new InstantCommand(() -> {
        testSwerveSubsystem.decrementTest();
        System.out.println(testSwerveSubsystem.getTest());
      }
      ));

      RBumper.onTrue(new InstantCommand(() -> {
        testSwerveSubsystem.incrementTest();
        System.out.println(testSwerveSubsystem.getTest());
      }
      ));

      AButton.onTrue(new InstantCommand(() -> {
        testSwerveSubsystem.toggletoRun();
        System.out.println(testSwerveSubsystem.getRunning() ? "Running" : "Not running");
      }));

    } else if(swerveSubsystem instanceof SingleModuleSwerveSubsystem){

      System.out.println("1");

      swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY(), .4 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
      }
      , swerveSubsystem));

      AButton.onTrue(new InstantCommand(() -> {
        swerveSubsystem.toggletoRun();
      }));
      
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
