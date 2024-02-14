package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {
    private final XboxController driveController = new XboxController(0);
    private final JoystickButton 
        driveAButton = new JoystickButton(driveController, XboxController.Button.kA.value),
        driveBButton = new JoystickButton(driveController, XboxController.Button.kB.value),
        driveXButton = new JoystickButton(driveController, XboxController.Button.kX.value),
        driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value),
        driveLBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value),
        driveRBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value),
        driveLStickButton = new JoystickButton(driveController, XboxController.Button.kLeftStick.value),
        driveRStickButton = new JoystickButton(driveController, XboxController.Button.kRightStick.value),
        driveBackButton = new JoystickButton(driveController, XboxController.Button.kBack.value),
        driveStartButton = new JoystickButton(driveController, XboxController.Button.kStart.value);

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

    public JoystickButton getFieldResetButton() {
        return driveAButton;
    }

    public JoystickButton getLeftBumper() {
        return driveLBumper;
    }

    public JoystickButton getRightBumper() {
        return driveRBumper;
    }

    public Boolean getRelativeMode() {
        return driveController.getRightTriggerAxis() > .1;
    }

    public JoystickButton getAmpAlign() {
        return driveXButton;
    }

    public JoystickButton getNoteAlign() {
        return driveYButton;
    }

    public JoystickButton getSwerveStop() {
        return driveBButton;
    }
}
