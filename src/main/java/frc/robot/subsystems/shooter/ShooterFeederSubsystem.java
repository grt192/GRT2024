package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;
import static frc.robot.Constants.ShooterConstants.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeederSubsystem extends SubsystemBase{

    //finals
    public final double FEEDER_MOTOR_SPEED = .8;
    public final int NO_NOTE_TOLERANCE = 500; //must test with no note in front of sensor
    public final int TOLERANCE = 1000; //represents the value when half note is in front of sensor

    //motors
    private final TalonSRX feederMotor; 

    //devices
    private final ColorSensorV3 shooterSensor; //distance sensor

    public ShooterFeederSubsystem(){
        //motors
        feederMotor = new TalonSRX(FEEDER_MOTOR_ID);
        feederMotor.setInverted(true);

        //sensors
        shooterSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    public void setFeederMotorSpeed(double speed){
        feederMotor.set(TalonSRXControlMode.PercentOutput, speed);
        System.out.println("feeding motor speed is: " + feederMotor.getMotorOutputPercent());

    }

    public int getRed(){
        // System.out.println("proximity: " + shooterSensor.getProximity());
        return shooterSensor.getRed();
    }

    public ColorSensorV3 getSensor(){
        System.out.println("returning shooter sensor");
        return shooterSensor;
    }

    @Override
    public void periodic() {
        // System.out.println(getRed());
    }
}
