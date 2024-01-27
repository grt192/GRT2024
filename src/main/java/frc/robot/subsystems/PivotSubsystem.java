package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase{

    //final vars
    public final double PIVOT_SPEED = 0.1;
    final double GEARBOX_RATIO = 18.16; //ask cadders
    public final int ERRORTOLERANCE = 5; //error tolerance for pid
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
    private boolean alliance; //true equals red alliance 
    private boolean autoAim;
    private double currentEncoderAngle;
    private Pose2d currentField = new Pose2d();

    //center of red speaker: (652.73 218.42)
    double RED_X = Units.inchesToMeters(652.73 + 9.05); //9.05 is half of 18.1 which is length of overhang of speaker-- we want halfway point
    double RED_Y = Units.inchesToMeters(218.42);

    //center of blue speaker: (-1.50 218.42)
    double BLUE_X = Units.inchesToMeters(-1.5+9.05); //9.05 is half of 18.1 which is length of overhang of speaker-- we want halfway point
    double BLUE_Y = Units.inchesToMeters(218.42);

    public PivotSubsystem(boolean alliance){

        //motors
        pivotMotor = new CANSparkMax(12, MotorType.kBrushless); 

        //devices
        rotationEncoder = pivotMotor.getEncoder();
        rotationEncoder.setPosition(0); 
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
        autoAim = false;
    }

    //motor speed setting functions
    public void setPivotMotorSpeed(double speed){
        pivotMotor.setVoltage(speed * 12);
        //System.out.println("flywheer motor speed is: " + pivotMotor.get());
    }


    public void setAngle(double angle){ //check if it works 
        rotationPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
        //System.out.println("setting angle to: " + angle);
    }

    public void setFieldPosition(Pose2d field){
        //System.out.println("setting field position");
        currentField = field;
    }

    public double getAutoAimAngle(double distance){
        double speakerHeight = Units.inchesToMeters(80.51);
        //System.out.println("Angle of shooter" + Math.atan(speakerHeight/distance));
        return Math.atan(speakerHeight/distance);
    }

    public double getDistance(){

        if(alliance){ //true = red
            double xLength = Math.pow(currentField.getX()-RED_X, 2);
            double yLength = Math.pow(currentField.getY()-RED_Y, 2);
            //System.out.println("alliance red:" + alliance);
            return Math.sqrt(xLength + yLength);

        } else {
            double xLength = Math.pow(currentField.getX()-BLUE_X, 2);
            double yLength = Math.pow(currentField.getY()-BLUE_Y, 2);

            return Math.sqrt(xLength + yLength);
        } 
    }

    public double getPosition(){
        //System.out.println("rotation encoder position: " + rotationEncoder.getPosition());
        return rotationEncoder.getPosition();
    }

    public double getCurrentAngle(){
        return currentEncoderAngle;
    }

    public void setAutoAimBoolean(boolean red){
        autoAim = red;
    }

    public void periodic(){
        //resets relative encoder every time robot starts again
        //(check if encoder prints zero when run)
        if(limitSwitch != null && limitSwitch.get()){ //false = limit switch is pressed
            rotationEncoder.setPosition(0); 
            // System.out.println(rotationEncoder.getPosition()); //should print 0
        }

        if(autoAim){
            setAngle(getAutoAimAngle(getDistance()));
        }

        // System.out.println("current pos" + rotationEncoder.getPosition());

        // if(currentState == ShooterState.FIRING && (shooterSensor.getRed() < TOLERANCE)){  //when there is no note
        //     setShooterState(ShooterState.NO_NOTE);
        // }
    }
}
