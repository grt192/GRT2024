package frc.robot.subsystems.swerve.drivemotors;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/** The REV Robotics Vortex version of SwerveDriveMotor. */
public class VortexDriveMotor implements SwerveDriveMotor {
    
    private CANSparkFlex motor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;
    private double conversionFactor = 1;
    private double lastReference;

    /** Vortex drive motor for swerve.
     *
     * @param canID The CAN ID of the SPARK Flex.
     */
    public VortexDriveMotor(int canID) {
        motor = new CANSparkFlex(canID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setClosedLoopRampRate(.1);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(conversionFactor); //STUB

        pidController = motor.getPIDController();
    }

    @Override
    public void setVelocity(double velocity) {
        lastReference = velocity;
        // motor.set(velocity / 4.9);
        pidController.setReference(velocity, ControlType.kVelocity);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public void configPID(double p, double i, double d, double ff)  {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(ff);
    }

    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void setVelocityConversionFactor(double factor) {
        encoder.setVelocityConversionFactor(factor);
        conversionFactor = factor;
    }

    @Override
    public void setPositionConversionFactor(double factor) {
        encoder.setPositionConversionFactor(factor);
    }

    @Override
    public double getError() {
        return getSetpoint() - encoder.getVelocity(); 
    }

    @Override
    public double getSetpoint() {
        return lastReference; 
    }

    @Override
    public double getAmpDraw() {
        return motor.getOutputCurrent(); 
    }

}
