package frc.robot.subsystems.swerve.drivemotors;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.MotorUtil;

public class VortexDriveMotor implements SwerveDriveMotor {
    
    private CANSparkFlex motor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;
    private double conversionFactor = 1;

    private double lastReference;

    public VortexDriveMotor (int port){
        motor = new CANSparkFlex(port, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(1); //STUB

        pidController = motor.getPIDController();

        // pidController = MotorUtil.createSparkPIDController(motor, encoder);
    }

    public void setVelocity(double velocity){
        lastReference = velocity;
        // motor.set(velocity / 4.9);
        pidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setPower(double power){
        motor.set(power);
    }

    public void configPID(double P, double I, double D, double FF){
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(FF);
    }

    public double getDistance(){
        return encoder.getPosition();
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void setVelocityConversionFactor(double factor){
        encoder.setVelocityConversionFactor(factor);
        conversionFactor = factor;
    }

    public void setPositionConversionFactor(double factor){
        encoder.setPositionConversionFactor(factor);
        System.out.println(" FACTOR " + factor);
    }

    public double getError(){
        return getSetpoint() - encoder.getVelocity(); 
    }

    public double getSetpoint(){
        return lastReference; //STUB
    }

    public double getSetPoint(){
        return 0; //STUB
    }

    public double getAmpDraw(){
        return motor.getOutputCurrent(); //STUB
    }
}
