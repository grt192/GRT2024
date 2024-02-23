package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/** controls shooter motors that shoot. */
public class ShooterFeederSubsystem extends SubsystemBase {

    //finals
    public final double FEEDER_MOTOR_SPEED = .8;
    public final int NO_NOTE_TOLERANCE = 500;
    public final int TOLERANCE = 1000; 
    private final double FEEDER_MOTOR_RESISTANCE = 1.01;

    //motors
    private final TalonFX feederMotor; 
    //private final TalonSRX feederMotor2;

    //devices
    private final ColorSensorV3 shooterSensor; //distance sensor

    /** Constructor to initialize motors and sensors. */
    public ShooterFeederSubsystem(){
        //motors
        feederMotor = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID);
        feederMotor.setInverted(true);

        // feederMotor2 = new TalonSRX(FEEDER_MOTOR_2_ID);
        // feederMotor.setInverted(true);

        //sensors
        shooterSensor = new ColorSensorV3(I2C.Port.kMXP);
    }

    /** Sets speed of shooting motors. */
    public void setFeederMotorSpeed(double speed){
        feederMotor.set(TalonFXControlMode.PercentOutput, speed);
        TalonFXControlMode
        //feederMotor2.set(TalonSRXControlMode.PercentOutput, speed*FEEDER_MOTOR_RESISTANCE);
        System.out.println("feeding motor speed is: " + feederMotor.getMotorOutputPercent());

    }

    /** Gets red value of anything in front of color sensor. */
    public int getRed(){
        // System.out.println("proximity: " + shooterSensor.getProximity());
        return shooterSensor.getRed();
    }

    /** Returns color sensor as an object. */
    public ColorSensorV3 getSensor(){
        System.out.println("returning shooter sensor");
        return shooterSensor;
    }

    @Override
    public void periodic() {
        // System.out.println(getRed());
    }
}
