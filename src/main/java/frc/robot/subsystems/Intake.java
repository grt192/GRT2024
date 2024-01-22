// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*To do:
 *  
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX motor1;
  private final WPI_TalonSRX topmotor;
  private final WPI_TalonSRX bottommotor;
  private final WPI_TalonSRX lastmotor;
  private final AnalogPotentiometer sensor;
  private final Encoder apple;
  private final DigitalInput boat;

  Boolean elevatorisdown;
  Boolean sensorreached;
  Boolean ampbutton;
  Boolean shooterbutton;


  /** Creates a new ExampleSubsystem. */
  public Intake() {
    motor1 = new WPI_TalonSRX(0);//big motor
    topmotor = new WPI_TalonSRX(1);
    bottommotor = new WPI_TalonSRX(2);
    lastmotor = new WPI_TalonSRX(3);
    sensor = new AnalogPotentiometer(4);
    apple = new Encoder(1,2);
    boat = new DigitalInput(0);
   
  }
  Intake arm = new Intake();
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

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  } /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
