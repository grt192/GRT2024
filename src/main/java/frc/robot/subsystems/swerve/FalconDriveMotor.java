package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconDriveMotor implements SwerveDriveMotor{
    
    private WPI_TalonFX motor;
    private double positionConversionFactor = 0;
    private double driveRotPerMinPerMetersPerSec = 0;

    public FalconDriveMotor(int port) {
        motor = new WPI_TalonFX(port);

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        motor.selectProfileSlot(0, 0);
    }

    public void setVelocity(double metersPerSec){

        double targetRpm = metersPerSec * driveRotPerMinPerMetersPerSec;

        double targetVelocityPer100ms = targetRpm * 2048 / 600; // (Rev / min) * (2048 units / Rev) * (min / 600 (100ms)) = units / 100ms

        // System.out.println(motor.getClosedLoopTarget() + " err: " + motor.getClosedLoopError());

        motor.set(TalonFXControlMode.Velocity, targetVelocityPer100ms);  
    }

    public void setPower(double power){
        motor.set(power);
    }

    public void configPID(double P, double I, double D, double FF){
        motor.config_kP(0, P);
        motor.config_kI(0, I);
        motor.config_kD(0, D);
        motor.config_kF(0, FF);
    }

    public double getDistance(){
        return motor.getSelectedSensorPosition() / positionConversionFactor;
    }

    public double getVelocity(){
        return motor.getSelectedSensorVelocity() / driveRotPerMinPerMetersPerSec; 
    }

    public void setVelocityConversionFactor(double factor){
        driveRotPerMinPerMetersPerSec = factor;
    }

    public void setPositionConversionFactor(double factor){
        positionConversionFactor = factor;
    }

}
