// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems;

import static frc.robot.Constants.RollerandPivotConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX topmotor;
  private final TalonFX bottommotor;
  private final TalonFX lastmotor;
  private final AnalogPotentiometer sensor;


  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    lastmotor = new TalonFX(lastmotorID);
    topmotor = new TalonFX(topmotorID);
    bottommotor = new TalonFX(bottommotorID);
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
    topmotor.set(top);
    bottommotor.set(bottom);
  }

  public void setAllRollSpeed(double topone, double bottomone){
    topmotor.set(topone);
    lastmotor.set(topone);
    bottommotor.set(bottomone);
  }

  public void setRollersOutwards(Boolean pressedA){
    if(pressedA==true)
      topmotor.set(rollersclockwise);
      bottommotor.set(rollerscounterclockwise);
  }

  public void setRollersInwards(Boolean pressedB){
    if(pressedB==true)
      topmotor.set(rollersclockwise);
      bottommotor.set(rollerscounterclockwise);
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

