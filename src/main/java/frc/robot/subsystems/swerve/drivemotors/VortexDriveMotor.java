package frc.robot.subsystems.swerve.drivemotors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.MotorUtil;

public class VortexDriveMotor implements SwerveDriveMotor {
    
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController pidController;

    public VortexDriveMotor (int port){
        motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(1); //STUB

        pidController = MotorUtil.createSparkPIDController(motor, encoder);
    }

    public void setVelocity(double velocity){
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
    }

    public void setPositionConversionFactor(double factor){
        encoder.setPositionConversionFactor(factor);
    }

    public double getError(){
        return 0; //STUB
    }
}