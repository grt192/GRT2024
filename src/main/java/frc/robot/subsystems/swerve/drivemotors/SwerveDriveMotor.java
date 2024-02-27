package frc.robot.subsystems.swerve.drivemotors;

/** Drive motors for driving swerve modules. */
public interface SwerveDriveMotor {
    
    /** Sets the velocity of the drive motor.
     *
     * @param velocity [~-4.5, ~4.5] Velocity in meters/sec
     */
    public void setVelocity(double velocity);

    /** Sets the raw power of the drive motor.
     *
     * @param power [-1., 1.] Raw power to set
     */
    public void setPower(double power);

    /** Configures the PID of the motor.
     *
     * @param p The proportional constant for the drive motor.
     * @param i The integral constant for the drive motor.
     * @param d The derivate constant for the drive motor.
     * @param ff The feed-forward constant for the drive motor.
     */
    public void configPID(double p, double i, double d, double ff);

    /** Gets the distance that the module has driven. Resets on code reboot.
     *
     * @return The distance driven in meters.
     */
    public double getDistance();

    /** Gets the current drive velocity in meters per second.
     *
     * @return The current velocity in meters/sec.
     */
    public double getVelocity();

    /** Sets the velocity conversion factor.
     *
     * @param factor The factor to multiply setpoints by when setting velocities.
     */
    public void setVelocityConversionFactor(double factor);
    
    /** Sets the position conversion factor.
     *
     * @param factor The factor to multiply positions by.
     */
    public void setPositionConversionFactor(double factor);

    /** Gets the velocity PID error.
     *
     * @return The current velocity PID error.
     */
    public double getError();

    /** Gets the velocity PID setpoint.
     *
     * @return The current velocity PID setpoint.
     */
    public double getSetpoint();

    /** Gets the current current draw of the drive motor.
     *
     * @return The current draw in amps.
     */
    public double getAmpDraw();
}
