package frc.robot.subsystems.swerve.drivemotors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class FalconDriveMotor implements SwerveDriveMotor{
    
    private TalonFX motor;
    private double positionConversionFactor = 0;
    private double driveRotPerMinPerMetersPerSec = 0;
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    private double targetRps = 0;

    public FalconDriveMotor(int port) {
        motor = new TalonFX(port);
    }

    public void setVelocity(double metersPerSec){

        targetRps = metersPerSec * driveRotPerMinPerMetersPerSec / 60; 

        // System.out.println(motor.getClosedLoopTarget() + " err: " + motor.getClosedLoopError());

        motor.setControl(request.withVelocity(targetRps )); //TODO: REMOVE LIMIT 
    }

    public void setPower(double power){
        motor.set(power);
    }

    public void configPID(double P, double I, double D, double FF){
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kV = FF;
        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;

        motor.getConfigurator().apply(slot0Configs);
    }

    public double getDistance(){
        return motor.getPosition().getValue() / positionConversionFactor;
    }

    public double getVelocity(){
        return motor.getVelocity().getValue() / driveRotPerMinPerMetersPerSec; 
    }

    public void setVelocityConversionFactor(double factor){
        driveRotPerMinPerMetersPerSec = factor;
    }

    public void setPositionConversionFactor(double factor){
        positionConversionFactor = factor;
    }

    public double getError(){
        return motor.getClosedLoopError().getValue();
    }

    public double getSetPoint(){
        return targetRps;
    }
    
    public double getAmpDraw(){
        return motor.getSupplyCurrent().getValueAsDouble();
    }

}
