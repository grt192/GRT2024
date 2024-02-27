package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A two-joystick drive controller on ports 0 and 1.
 */
public class DualJoystickDriveController extends BaseDriveController {

    // TODO: Better names for these
    private final Joystick leftJoystick = new Joystick(0);
    private final JoystickButton leftTrigger = new JoystickButton(leftJoystick, 1);
    private final JoystickButton leftMiddleButton = new JoystickButton(leftJoystick, 2);
    private final JoystickButton leftTopLeftButton = new JoystickButton(leftJoystick, 3);
    private final JoystickButton leftTopRightButton = new JoystickButton(leftJoystick, 4);

    private final Joystick rightJoystick = new Joystick(1);
    private final JoystickButton rightTrigger = new JoystickButton(rightJoystick, 1);
    private final JoystickButton rightTopButton = new JoystickButton(rightJoystick, 3);
    private final JoystickButton rightMiddleLeftButton = new JoystickButton(rightJoystick, 5);
    private final JoystickButton rightMiddleRightButton = new JoystickButton(rightJoystick, 6);

    private static final double JOYSTICK_DEADBAND = 0.08;

    @Override
    public double getForwardPower() {
        double scale = getDriveScaling();
        return MathUtil.applyDeadband(-leftJoystick.getY() * scale, JOYSTICK_DEADBAND);
    }

    @Override
    public double getLeftPower() {
        double scale = getDriveScaling();
        return MathUtil.applyDeadband(-leftJoystick.getX() * scale, JOYSTICK_DEADBAND);
    }

    @Override
    public double getRotatePower() {
        return MathUtil.applyDeadband(-rightJoystick.getX() * getTurnScaling(), JOYSTICK_DEADBAND);
    }

    /** Gets the amount to scale translational input by.
     *
     * @return The scale to apply to translational input.
     */
    private double getDriveScaling() {
        boolean isSlowMode = leftJoystick.getTrigger();
        return isSlowMode ? 0.24 : 1.0;
    }

    /** Gets the amount to scale rotational input by.
     *
     * @return The scale to apply to rotational input.
     */
    private double getTurnScaling() {
        boolean isSlowMode = leftJoystick.getTrigger();
        return isSlowMode ? 0.125 : 0.8;
    }

    @Override
    public JoystickButton getDriverHeadingResetButton() {
        return rightTopButton;
    }

    @Override
    public JoystickButton getRightBumper() {
        return rightMiddleRightButton;
    }

    @Override
    public JoystickButton getLeftBumper() {
        return rightMiddleLeftButton;
    }

    @Override
    public Boolean getRelativeMode() {
        return rightTrigger.getAsBoolean();
    }

    @Override
    public JoystickButton getAmpAlign() {
        return leftTopLeftButton;
    }

    @Override
    public JoystickButton getNoteAlign() {
        return leftMiddleButton;
    }

    @Override
    public JoystickButton getSwerveStop() {
        return leftTrigger;
    }
    
    @Override
    public Boolean getSwerveAimMode() {
        return leftTopRightButton.getAsBoolean();
    }
    
}
