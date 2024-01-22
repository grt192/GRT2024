// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*To do:
 *  
 */

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX motor1;
  private final TalonFX topmotor;
  private final TalonFX bottommotor;
  private final TalonFX lastmotor;
  private final AnalogPotentiometer sensor;
  private final Encoder apple;
  private final DigitalInput boat;

  Boolean elevatorisdown;
  Boolean sensorreached;
  Boolean ampbutton;
  Boolean shooterbutton;


  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    motor1 = new TalonFX(0);//big motor
    topmotor = new TalonFX(1);
    bottommotor = new TalonFX(2);
    lastmotor = new TalonFX(3);
    sensor = new AnalogPotentiometer(4);
    apple = new Encoder(1,2);
    boat = new DigitalInput(0);
   
  }
  IntakeSubsystem arm = new IntakeSubsystem();
  Boolean cherry=false;
  Boolean berry=false;
  
  public void retract(){
    while(apple.get() != 2){
     motor1.set(1);
    }
  }

  public void extend(){
    while(boat.get()==false){//nooo
      motor1.set(2);
    }
  }

  @Override
  public void periodic() {

  
    // This method will be called once per scheduler run

    //something limit switch set zero
    if (boat.get()){
      apple.reset();
    }

    else if (elevatorisdown && shooterbutton && sensorreached){//to shooter
      topmotor.set(2);
      bottommotor.set(1);
      lastmotor.set(2);
      //somehow make it go longer
    }
    
    else if (elevatorisdown && sensorreached==false){//intake
      arm.extend(); //make a thing
      topmotor.set(1);
      bottommotor.set(2);
      cherry = true;
    }

    else if (cherry == true){//after intake
      arm.retract();
      cherry = false;
    }

    else if (elevatorisdown==false && ampbutton && sensorreached){//amp
      arm.extend();
      topmotor.set(1);
      bottommotor.set(2);
    //make it run a littl emore 
      berry = true;
    }

    else if (berry == true){//after amp}
      arm.retract();
      berry = false;
    }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
