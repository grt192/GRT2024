// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollersSubsystem extends SubsystemBase {
  private final TalonSRX frontMotor;
  private final TalonSRX backMotor;
  private final TalonSRX integrationMotor;
  private final AnalogPotentiometer sensor;


  /** Creates a new ExampleSubsystem. */
  public IntakeRollersSubsystem() {
    integrationMotor = new TalonSRX(INTEGRATION_MOTOR_ID);
    frontMotor = new TalonSRX(FRONT_MOTOR_ID);
    frontMotor.setInverted(true);
    backMotor = new TalonSRX(BACK_MOTOR_ID);
    sensor = new AnalogPotentiometer(SENSOR_ID);
  }
  
  public boolean sensorNow(){
    if (sensor.get()>=SENSOR_REACHED){
      return true;
    }
    else{
      return false;
    }
  }

  public void setRollSpeed(double top, double bottom){
    frontMotor.set(TalonSRXControlMode.PercentOutput, top);
    backMotor.set(TalonSRXControlMode.PercentOutput, bottom);
  }

  public void setAllRollSpeed(double topone, double bottomone){
    frontMotor.set(TalonSRXControlMode.PercentOutput, topone);
    integrationMotor.set(TalonSRXControlMode.PercentOutput, bottomone);
    backMotor.set(TalonSRXControlMode.PercentOutput, topone);
  }

  public void setRollersOutwards(Boolean pressedA){
    if(pressedA==true)
      frontMotor.set(TalonSRXControlMode.PercentOutput, ROLLERS_CLOCKWISE);
      backMotor.set(TalonSRXControlMode.PercentOutput, ROLLERS_COUNTERCLOCKWISE);
  }

  public void setRollersInwards(Boolean pressedB){
    if(pressedB==true)
      frontMotor.set(TalonSRXControlMode.PercentOutput, ROLLERS_CLOCKWISE);
      backMotor.set(TalonSRXControlMode.PercentOutput, ROLLERS_COUNTERCLOCKWISE);
  }
  

  @Override
  public void periodic() {
    // System.out.println("intake distance sensor: " + sensor.get());
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

