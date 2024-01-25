// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX topmotor;
  private final TalonFX bottommotor;
  private final TalonFX lastmotor;
  private final AnalogPotentiometer sensor;

  Integer sensorreached;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    lastmotor = new TalonFX(0);
    topmotor = new TalonFX(1);
    bottommotor = new TalonFX(2);
    sensor = new AnalogPotentiometer(4);
  }
  
  public boolean sensornow(){
    if (sensor.get()<=sensorreached){
      return true;
    }
    else{
      return false;
    }
  }

  public void setRollSpeed(double top, double bottom){
    topmotor.set(top);
    bottommotor.set(bottom);
  }

  public void setRollSpeedTwo(double topone, double bottomone){
    topmotor.set(topone);
    lastmotor.set(topone);
    bottommotor.set(bottomone);
  }

  public void setRollersOutwards(Boolean pressedA){
    if(pressedA==true)
      topmotor.set(1);
      bottommotor.set(-1);
  }

  public void setRollersInwards(Boolean pressedB){
    if(pressedB==true)
      topmotor.set(-1);
      bottommotor.set(1);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //something limit switch set zero
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

