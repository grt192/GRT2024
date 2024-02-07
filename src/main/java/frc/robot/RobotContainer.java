// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimbIdleCommand;
import frc.robot.commands.ClimbLowerCommand;
import frc.robot.commands.ClimbRaiseCommand;
import frc.robot.subsystems.TestMotorSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.ClimbConstants;

public class RobotContainer {
  // private final BaseSwerveSubsystem baseSwerveSubsystem;
  private final boolean IS_MANUAL = true;
  
  private final TestMotorSubsystem testClimbLeft;
  private final TestMotorSubsystem testClimbRight;
  private final ClimbSubsystem climbSubsystem;

  private final XboxController controller = new XboxController(2);

  private final JoystickButton
    YButton = new JoystickButton(controller, XboxController.Button.kY.value),
    AButton = new JoystickButton(controller, XboxController.Button.kA.value),
    BButton = new JoystickButton(controller, XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (!IS_MANUAL) {
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
