// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems.shooter;
import static frc.robot.Constants.ShooterConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFlywheelSubsystem extends SubsystemBase {

    //change later (ALL OF THEM ARE PLACEHOLDERS)
    int IDNUMBER = 10; //so I remember to change them later
    // public static final int SHOOTER_MOTOR_ONE_ID = 13;
    // public static final int SHOOTER_MOTOR_TWO_ID = 14;
    public final double SHOOTER_MOTOR_SPEED = 1;

    //motors
    private final CANSparkMax shooterMotor;
    private final CANSparkMax shooterMotorTwo;

    //devices
    private ShooterState currentState; //an enum state thing

    //sensors

    public ShooterFlywheelSubsystem(){
        //motors
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ONE_ID, MotorType.kBrushless);
        shooterMotorTwo = new CANSparkMax(SHOOTER_MOTOR_TWO_ID, MotorType.kBrushless);

        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotorTwo.setIdleMode(IdleMode.kCoast);
        

        //second motor shooter follows first
        shooterMotorTwo.follow(shooterMotor, true);

        //enums
        setShooterState(ShooterState.VERTICAL);
    }

    //enum functions
    public ShooterState getShooterState(ShooterState state){
        return currentState;
    }

    public void setShooterState(ShooterState newState){
        currentState = newState;
    }

    public void setShooterMotorSpeed(double speed){
        shooterMotor.setVoltage(-speed * 12);
        System.out.println("shooter motor speed is: " + shooterMotor.get());
    }

}