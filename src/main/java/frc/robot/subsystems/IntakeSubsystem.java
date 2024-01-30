// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems;

import static frc.robot.Constants.RollerandPivotConstants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX topmotor;
  private final TalonSRX bottommotor;
  private final TalonSRX lastmotor;
  private final AnalogPotentiometer sensor;


  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    lastmotor = new TalonSRX(lastmotorID);
    topmotor = new TalonSRX(topmotorID);
    bottommotor = new TalonSRX(bottommotorID);
    sensor = new AnalogPotentiometer(sensorID);
  }
  
  public boolean sensorNow(){
    if (sensor.get()<=sensorreached){
      return true;
    }
    else{
      return false;
    }
  }

  public void setRollSpeed(double top, double bottom){
    topmotor.set(TalonSRXControlMode.PercentOutput, top);
    bottommotor.set(TalonSRXControlMode.PercentOutput, bottom);
  }

  public void setAllRollSpeed(double topone, double bottomone){
    topmotor.set(TalonSRXControlMode.PercentOutput, topone);
    lastmotor.set(TalonSRXControlMode.PercentOutput, topone);
    bottommotor.set(TalonSRXControlMode.PercentOutput, bottomone);
  }

  public void setRollersOutwards(Boolean pressedA){
    if(pressedA==true)
      topmotor.set(TalonSRXControlMode.PercentOutput, rollersclockwise);
      bottommotor.set(TalonSRXControlMode.PercentOutput, rollerscounterclockwise);
  }

  public void setRollersInwards(Boolean pressedB){
    if(pressedB==true)
      topmotor.set(TalonSRXControlMode.PercentOutput, rollersclockwise);
      bottommotor.set(TalonSRXControlMode.PercentOutput, rollerscounterclockwise);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

