package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

public class PivotSubsystem {

    //final vars
    public final double PIVOT_SPEED = 0.1;
    final double GEARBOX_RATIO = 1/20; //ask cadders
    public final int ERRORTOLERANCE = 10; //error tolerance for pid
    final int LIMIT_SWITCH_ID = 1; //placeholder

    //motors
    private final CANSparkMax pivotMotor;

    //devices
    private RelativeEncoder rotationEncoder;
    private SparkPIDController rotationPIDController;
    private final DigitalInput limitSwitch;

    //angle PID (CHANGE LATER)
    private static final double ANGLE_P = 2.4;
    private static final double ANGLE_I = 0;
    private static final double ANGLE_D = 0;

    public PivotSubsystem(){

        //motors
        pivotMotor = new CANSparkMax(10, MotorType.kBrushless); 

        //devices
        rotationEncoder = pivotMotor.getEncoder();
        rotationPIDController = pivotMotor.getPIDController();
        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);


        //setting PID vars
        rotationPIDController.setP(ANGLE_P);
        rotationPIDController.setI(ANGLE_I);
        rotationPIDController.setD(ANGLE_D);

        //encoder stuff
        rotationEncoder.setPositionConversionFactor(GEARBOX_RATIO);
        rotationPIDController.setSmartMotionAllowedClosedLoopError(ERRORTOLERANCE, 0); //what does 0 do (slotID is from 0-3)

    }

    //motor speed setting functions
    public void setPivotMotorSpeed(double speed){
        pivotMotor.setVoltage(speed * 12);
        System.out.println("flywheer motor speed is: " + pivotMotor.get());
    }


    public void setAngle(double angle){ //check if it works 
        rotationPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
        System.out.println("setting angle to: " + angle);
    }

    public double getPosition(){
        return rotationEncoder.getPosition();
    }

    public void periodic(){
        //resets relative encoder every time robot starts again
        //(check if encoder prints zero when run)
        if(limitSwitch != null && limitSwitch.get()){ //false = limit switch is pressed
            rotationEncoder.setPosition(0); 
            System.out.println(rotationEncoder.getPosition()); //should print 0
        }

        // if(currentState == ShooterState.FIRING && (shooterSensor.getRed() < TOLERANCE)){  //when there is no note
        //     setShooterState(ShooterState.NO_NOTE);
        // }
    }
}
