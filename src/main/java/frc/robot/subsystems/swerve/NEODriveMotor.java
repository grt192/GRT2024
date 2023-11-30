package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.MotorUtil;

public class NEODriveMotor implements SwerveDriveMotor {
    
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;

    public NEODriveMotor (int port){
        motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(1); //STUB

        pidController = MotorUtil.createSparkMaxPIDController(motor, encoder);
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
}
