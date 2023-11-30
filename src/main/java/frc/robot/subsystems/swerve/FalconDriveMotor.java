package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconDriveMotor implements SwerveDriveMotor{
    
    private WPI_TalonFX motor;

    public FalconDriveMotor(int port) {
        motor = new WPI_TalonFX(port);
    }

    public void setVelocity(double velocity){
        motor.set(ControlMode.Velocity, velocity);  //is this rpm? - test (also does this use the pid correctly?)
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
        return motor.getSelectedSensorPosition(); //STUB need to convert to meters
    }

    public double getVelocity(){
        return motor.getSelectedSensorVelocity(); //STUB need to convert to meters/sec
    }

}
