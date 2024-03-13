package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A singular swerve module. */
public class SingleModuleSwerveSubsystem extends SubsystemBase {

    private static final double MAX_VEL = 5; // m/s STUB

    private Timer crimor;
    private boolean toRun;
    
    SwerveModule module;

    private double angle = 0;

    /** A singular swerve module.
     *
     * @param module The module to use.
     */
    public SingleModuleSwerveSubsystem(SwerveModule module) {
        toRun = false;
        this.module = module;
        crimor = new Timer();
        crimor.start();
    }

    /** Set the raw powers of the module.
     *
     * @param drivePower [-1, 1] Drive motor power.
     * @param steerPower [-1, 1] Steer motor power.
     */
    public void setRawPowers(double drivePower, double steerPower) {
        module.setRawPowers(drivePower, steerPower);
    }

    /** Set the raw power of drive with a set angle.
     *
     * @param drivePower [-1, 1] The power to set the drive motor to.
     * @param angleRads [-pi, pi] The angle in radians to set the steer to.
     */
    public void setRawPowersWithAngle(double drivePower, double angleRads) {
        module.setRawPowersWithAngle(drivePower, angleRads);
    }

    /** Set the drive powers. xPower is forward and yPower is left.
     *
     * @param xPower [-1, 1] The power in the x direction.
     * @param yPower [-1, 1] The power in the y direction.
     */
    public void setDrivePowers(double xPower, double yPower) {
        double velocity = MAX_VEL * Math.sqrt(yPower * yPower + xPower * xPower) / Math.sqrt(2);
        double angle = Math.atan2(yPower, xPower);

        module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
    }

    /** Set the drive powers. x and y are used for angle, drivePower is for drive.
     *
     * @param x [-1, 1] The x direction for setting the angle.
     * @param y [-1, 1] The y direction for setting the angle.
     * @param drivePower [-1, 1] The power to run the drive motor at.
     */
    public void setDrivePowers(double x, double y, double drivePower) {
        if (!(x == 0 && y == 0)) {
            angle = Math.atan2(y, x);
        }
        double velocity = MAX_VEL * drivePower;
        if (toRun) {
            module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
        }
    }

    /** Toggles whether the module should run or not. */
    public void toggleToRun() {
        toRun = !toRun;
    }
}
