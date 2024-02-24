package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {
    private final XboxController driveController = new XboxController(0);
     
    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driveController, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton lBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rBumper = new JoystickButton(
        driveController, 
        XboxController.Button.kRightBumper.value
    );
    private final JoystickButton driveLStickButton = new JoystickButton(
        driveController, XboxController.Button.kLeftStick.value
    );

    @Override
    public double getForwardPower() {
        return -driveController.getLeftY();
    }

    @Override
    public double getLeftPower() {
        return -driveController.getLeftX();
    }

    @Override
    public double getRotatePower() {
        return -driveController.getRightX();
    }

    @Override
    public JoystickButton getDriverHeadingResetButton() {
        return aButton;
    }

    @Override
    public JoystickButton getLeftBumper() {
        return lBumper;
    }

    @Override
    public JoystickButton getRightBumper() {
        return rBumper;
    }

    @Override
    public Boolean getRelativeMode() {
        return driveController.getRightTriggerAxis() > .1;
    }

    @Override
    public JoystickButton getAmpAlign() {
        return xButton;
    }

    @Override
    public JoystickButton getNoteAlign() {
        return yButton;
    }

    @Override
    public JoystickButton getSwerveStop() {
        return bButton;
    }

    @Override
    public Boolean getSwerveAimMode() {
        return driveLStickButton.getAsBoolean();
    }
}
