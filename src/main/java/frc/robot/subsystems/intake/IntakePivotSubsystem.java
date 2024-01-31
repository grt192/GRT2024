package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
//import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakePivotSubsystem extends SubsystemBase{
    private final com.ctre.phoenix.motorcontrol.can.TalonSRX pivotMotor;
    // private final Encoder intakeencoder;
    private final DigitalInput extendedlimitswitch;
    private final DigitalInput retractedlimitswitch;

    public IntakePivotSubsystem(){
        pivotMotor = new TalonSRX(PIVOT_MOTOR_ID);
        //intakeencoder = new Encoder(1,2);
        extendedlimitswitch = new DigitalInput(extendedlimitswitchID);
        retractedlimitswitch = new DigitalInput(retractedlimitswitchID);
    }
   
    // public void resetEncoder(){
    //     intakeencoder.reset();
    // }

    // public double encoderPosition(){
    //     return intakeencoder.get();
    // }

    public void movePivot(double speed){
        pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public boolean pivotisextended(){
        if (extendedlimitswitch.get()){
          return true;
        }
        else{
          return false;
        }
    }

    public boolean pivotisretracted(){
        if(retractedlimitswitch.get()){
          return true;
        }
        else{
          return false;
        }
    }

    public void setPivotSpeed(double right){
        pivotMotor.set(TalonSRXControlMode.PercentOutput, right);
      }

    
    

    










 

}