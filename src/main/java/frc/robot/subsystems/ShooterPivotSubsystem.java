package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

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

    //field
    private Pose2d field;
    private boolean alliance; //true equals red alliance 

    public ShooterPivotSubsystem(boolean alliance){

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

        //field
        this.alliance = alliance;
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

    public void setFieldPosition(Pose2d field){
        double speakerHeight = 80.51;

        //center of red speaker: (652.73 218.42)
        double redX = 652.73 + 9.05; //9.05 is half of 18.1 which is length of overhang of speaker-- we want halfway point
        double redY = 218.42;

        //center of blue: (-1.50 218.42)
        double blueX = -1.5+9.05; //9.05 is half of 18.1 which is length of overhang of speaker-- we want halfway point
        double blueY = 218.42;

        if(alliance){ //true = red
            double dist = getDistance(field.getX(), field.getY(), redX, redY);
            setAngle(Math.atan(speakerHeight/dist));
        } else if (!alliance){
            double dist = getDistance(field.getX(), field.getY(), blueX, blueY);
            setAngle(Math.atan(speakerHeight/dist));
        }   
    }

    public double getDistance(double robotX, double robotY, double speakerX, double speakerY){
        double xLength = Math.pow(robotX-speakerX, 2);
        double yLength = Math.pow(robotY-speakerY, 2);

        return Math.sqrt(xLength + yLength);
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
