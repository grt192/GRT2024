package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** The base class for a drive controller. Contains all needed methods for driving the robot (without mechs) */
public abstract class BaseDriveController {

    /**
     * Gets the forward power commanded by the controller.
     *
     * @return The [-1.0, 1.0] forward power.
     */
    public abstract double getForwardPower();

    /**
     * Gets the left power commanded by the controller.
     *
     * @return The [-1.0, 1.0] left power.
     */
    public abstract double getLeftPower();

    /**
     * Gets the rotational power commanded by the controller.
     *
     * @return The [-1.0, 1.0] angular power.
     */
    public abstract double getRotatePower();

    /**
     * Gets the button to reset the driver heading.
     *
     * @return The JoystickButton to reset the driver heading. 
     */
    public abstract JoystickButton getDriverHeadingResetButton();

    /**
     * Gets the left bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the left bumper or equivalent.
     */
    public abstract JoystickButton getLeftBumper();

    /**
     * Gets the right bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the right bumper or equivalent.
     */
    public abstract JoystickButton getRightBumper();

    /**
     * Gets whether the driver is currently running in relative mode or not. Relative mode means moving the joystick
     * forward will move the robot in the direction of the intake. The default driving mode is field relative, meaning
     * forward is a direction set on the field by the driver (see resetDriverHeading)
     *
     * @return true if relativeMode is active, false otherwise
     */
    public abstract Boolean getRelativeMode();

    /**
     * Gets the button bound to auto-aligning to the amp.
     *
     * @return The JoystickButton to ampAlign.
     */
    public abstract JoystickButton getAmpAlign();

    /**
     * Gets the button bound to run the noteAlign sequence.
     *
     * @return The Joystick button to noteAlign.
     */
    public abstract JoystickButton getNoteAlign();

    /**
     * Gets the button bound to stop the swerve from any automatic motion.
     *
     * @return The button to stop swerve.
     */
    public abstract JoystickButton getSwerveStop();

    /**
     * Gets whether swerveAim should be used or not.
     *
     * @return true if swerveAim should be active, false otherwise.
     */
    public abstract Boolean getSwerveAimMode();

    /**
     * Gets the button to aim the shooter at the speaker.
     */
    public abstract JoystickButton getShooterAimButton();
}
