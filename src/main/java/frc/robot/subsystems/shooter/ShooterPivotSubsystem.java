package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.Pose2dSupplier;
import frc.robot.util.Util;

/** Controls motors and functions for the pivot part of shooter mech. */
public class ShooterPivotSubsystem extends SubsystemBase {

    //motors
    private final CANSparkMax pivotMotor;

    //devices
    private RelativeEncoder rotationEncoder;
    private SparkPIDController rotationPIDController;
    private final DigitalInput limitSwitch;

    //angle PID (CHANGE LATER
    private static final double ANGLE_P = 1;
    private static final double ANGLE_I = 0;
    private static final double ANGLE_D = 0;

    //field
    private boolean alliance; //true equals red alliance 
    private boolean autoAim;
    private double currentEncoderAngle;
    private double currentDistance;
    private Pose2dSupplier poseSupplier; //new Pose2d();

    private final Timer timer = new Timer();

    /** Inits motors and pose field. Also inits PID stuff. */
    public ShooterPivotSubsystem(boolean alliance, Pose2dSupplier poseSupplier){

        timer.start();
        this.poseSupplier = poseSupplier;

        //motors
        pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_MOTOR_ID, MotorType.kBrushless); 
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(false);

        //devices
        rotationEncoder = pivotMotor.getEncoder();
        rotationPIDController = pivotMotor.getPIDController();
        rotationPIDController.setOutputRange(-.3, 0.6);
        limitSwitch = new DigitalInput(ShooterConstants.LIMIT_SWITCH_ID);


        //setting PID vars
        rotationPIDController.setP(ANGLE_P);
        rotationPIDController.setI(ANGLE_I);
        rotationPIDController.setD(ANGLE_D);
        rotationPIDController.setFF(0); 
        rotationPIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.PID_ERROR_TOLERANCE, 0); 

        //encoder stuff
        rotationEncoder.setPositionConversionFactor(ShooterConstants.CONVERSION_FACTOR);
        rotationEncoder.setVelocityConversionFactor(ShooterConstants.CONVERSION_FACTOR * 60);
        rotationEncoder.setPosition(Units.degreesToRadians(18));
        rotationPIDController.setSmartMotionAllowedClosedLoopError(ShooterConstants.PID_ERROR_TOLERANCE, 0); 

        //pivot soft limits
        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRadians(62));
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Units.degreesToRadians(18));
        pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        

        //field
        this.alliance = alliance;
        autoAim = false;

        rotationPIDController.setReference(Units.degreesToRadians(18), ControlType.kPosition);
    }
    
    /** motor speed setting functions. */
    public void setPivotMotorSpeed(double speed) {
        pivotMotor.setVoltage(speed * 12);
        //System.out.println("flywheer motor speed is: " + pivotMotor.get());
    }

    /** Sets Angle of the pivot.*/
    public void setAngle(double angle) { 
        rotationPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
        System.out.println("setting angle to: " + angle);
       
    }

    /** Gets correct Angle for pivot to turn to. */
    public double getAutoAimAngle() {
        double speakerHeight = Units.inchesToMeters(80.51);
        Pose2d currentField = poseSupplier.getPose2d();
        //System.out.println("Angle of shooter" + Math.atan(speakerHeight/distance));

        if (alliance) {  //true = red
            double xLength = Math.pow(currentField.getX() - ShooterConstants.RED_X, 2);
            double yLength = Math.pow(currentField.getY() - ShooterConstants.RED_Y, 2);
            currentDistance = Math.sqrt(xLength + yLength);

        } else {
            double xLength = Math.pow(currentField.getX() - ShooterConstants.BLUE_X, 2);
            double yLength = Math.pow(currentField.getY() - ShooterConstants.BLUE_Y, 2);

            currentDistance = Math.sqrt(xLength + yLength);

            System.out.println(currentDistance);
        }
        
        if (currentDistance < 1.75) {
            return Units.degreesToRadians(62);
        }

        return Math.atan(speakerHeight / currentDistance) + Units.degreesToRadians(5);
    }

    /** Prints pivot current angle. */
    public void printCurrentAngle() {
        // System.out.println("radians: " + rotationEncoder.getPosition());
        // System.out.println(pivotMotor.get());
        // System.out.println(rotationPIDController.getFF());
    }

    /** Gets position of encoder. */
    public double getPosition() {
        //System.out.println("rotation encoder position: " + rotationEncoder.getPosition());
        return rotationEncoder.getPosition();
    }

    /** Gets encoder's current angle. */
    public double getCurrentAngle() {
        return currentEncoderAngle;
    }

    /** Sets whether Auto-aim should be on or off. */
    public void setAutoAimBoolean(boolean autonAim) { 
        this.autoAim = autonAim;
    }

    @Override
    public void periodic() {
        //resets relative encoder every time robot starts again
        //(check if encoder prints zero when run)
        // if(limitSwitch != null && limitSwitch.get()){ //false = limit switch is pressed
        //     rotationEncoder.setPosition(0); 
        //     // System.out.println(rotationEncoder.getPosition()); //should print 0
        // }

        if (autoAim) {
            setAngle(getAutoAimAngle());
        }


        if (timer.advanceIfElapsed(.2)) { 
            //printCurrentAngle();
            //System.out.println(Util.twoDecimals(Units.radiansToDegrees(getAutoAimAngle())));
            // System.out.println(Util.twoDecimals(Units.radiansToDegrees(getAutoAimAngle())));
        }

        // System.out.println("current pos" + rotationEncoder.getPosition());

        // if(currentState == ShooterState.FIRING && (shooterSensor.getRed() < TOLERANCE)){  //when there is no note
        //     setShooterState(ShooterState.NO_NOTE);
        // }
    }
}
