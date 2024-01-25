// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ElevatorConstants.ZERO_LIMIT_ID;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.BaseSwerveSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TestSingleModuleSwerveSubsystem;

public class RobotContainer {
  private final BaseSwerveSubsystem baseSwerveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
      
  private final XboxController controller = new XboxController(0);
  // private final SwerveModule module;

  private final JoystickButton
    LBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value),
    RBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value),
    AButton = new JoystickButton(controller, XboxController.Button.kA.value);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  private final XboxController mechController = new XboxController(Constants.OperatorConstants.kDriverControllerPort); 

  public RobotContainer() {
    //construct Test
    // module = new SwerveModule(0, 1, 0);
    // baseSwerveSubsystem = new SingleModuleSwerveSubsystem(module);
    baseSwerveSubsystem = new SwerveSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    // Configure the trigger bindings
    //configureBindings();    
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
    
    elevatorSubsystem.setDefaultCommand(new InstantCommand(() -> {
        if(mechController.getLeftTriggerAxis() != 0 && mechController.getRightTriggerAxis() != 0){
          elevatorSubsystem.setManual();
          elevatorSubsystem.setManualPower(mechController.getRightTriggerAxis()-mechController.getLeftTriggerAxis());
        }
        else{
          elevatorSubsystem.setAuto();
        }
    }));
    if(baseSwerveSubsystem instanceof SwerveSubsystem){
      final SwerveSubsystem swerveSubsystem = (SwerveSubsystem) baseSwerveSubsystem;

      swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY(), -controller.getRightX());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
      }
      , swerveSubsystem));

      AButton.onTrue(new InstantCommand(() -> {
        swerveSubsystem.resetDriverHeading();
      }
      ));
      
    } else if(baseSwerveSubsystem instanceof TestSingleModuleSwerveSubsystem){
      final TestSingleModuleSwerveSubsystem testSwerveSubsystem = (TestSingleModuleSwerveSubsystem) baseSwerveSubsystem;
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

    } else if (baseSwerveSubsystem instanceof SingleModuleSwerveSubsystem){
      final SingleModuleSwerveSubsystem swerveSubsystem = (SingleModuleSwerveSubsystem) baseSwerveSubsystem;

      System.out.println("1");

      swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(controller.getLeftX(), -controller.getLeftY());//, 1 * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
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
