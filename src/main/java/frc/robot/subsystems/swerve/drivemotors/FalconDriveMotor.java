package frc.robot.subsystems.swerve.drivemotors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

/** A falcon drive motor for swerve. Deprecated for R2 */
public class FalconDriveMotor implements SwerveDriveMotor {
    
    private TalonFX motor;
    private double positionConversionFactor = 0;
    private double driveRotPerMinPerMetersPerSec = 0;
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    private double targetRps = 0;

    /** A falcon drive motor for swerve.
     *
     * @param canId The canId of the motor.
     */
    public FalconDriveMotor(int canId) {
        motor = new TalonFX(canId);
    }

    @Override
    public void setVelocity(double metersPerSec) {
        targetRps = metersPerSec * driveRotPerMinPerMetersPerSec / 60; 
        motor.setControl(request.withVelocity(targetRps));
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void configPID(double p, double i, double d, double ff) {
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kV = ff;
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;

        motor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public double getDistance() {
        return motor.getPosition().getValue() / positionConversionFactor;
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValue() / driveRotPerMinPerMetersPerSec; 
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        driveRotPerMinPerMetersPerSec = factor;
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        positionConversionFactor = factor;
    }

    @Override
    public double getError() {
        return motor.getClosedLoopError().getValue();
    }

    @Override
    public double getSetpoint() {
        return targetRps;
    }
    
    @Override
    public double getAmpDraw() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }
}
