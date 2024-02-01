package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeederSubsystem extends SubsystemBase{

    //finals
    public final double FEEDER_MOTOR_SPEED = 0.1;
    public final int NO_NOTE_TOLERANCE = 10; //must test with no note in front of sensor
    public final int TOLERANCE = 10; //represents the value when half note is in front of sensor

    //motors
    private final TalonSRX feederMotor; 

    //devices
    private final ColorSensorV3 shooterSensor; //distance sensor

    public ShooterFeederSubsystem(){
        //motors
        feederMotor = new TalonSRX(15);

        feederMotor.setInverted(true);

        //sensors
        shooterSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    public void setFeederMotorSpeed(double speed){
        feederMotor.set(TalonSRXControlMode.PercentOutput, speed);
        System.out.println("feeding motor speed is: " + feederMotor.getMotorOutputPercent());

    }

    public int getProximity(){
        System.out.println("proximity: " + shooterSensor.getProximity());
        return shooterSensor.getProximity();
    }

    public ColorSensorV3 getSensor(){
        System.out.println("returning shooter sensor");
        return shooterSensor;
    }
}
