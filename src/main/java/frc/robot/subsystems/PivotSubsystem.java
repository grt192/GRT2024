package frc.robot.subsystems;

import static frc.robot.Constants.RollerandPivotConstants.*;
//import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotSubsystem extends SubsystemBase{
    private final TalonFX motor1;
    // private final Encoder intakeencoder;
    private final DigitalInput extendedlimitswitch;
    private final DigitalInput retractedlimitswitch;

    public PivotSubsystem(){
        motor1 = new TalonFX(motor1ID);
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
        motor1.set(speed);
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

    public void setPivotSpeed(double right, double left){
        motor1.set(right+left);
      }

    
    

    










 

}