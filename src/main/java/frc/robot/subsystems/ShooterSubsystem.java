// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterState;

public class ShooterSubsystem extends SubsystemBase {

    //change later (ALL OF THEM ARE PLACEHOLDERS)
    int IDNUMBER = 10; //so I remember to change them later
    public final double SHOOTER_MOTOR_SPEED = 0.1;

    //motors
    private final CANSparkMax shooterMotor;
    private final CANSparkMax shooterMotorTwo;

    //devices
    private ShooterState currentState; //an enum state thing

    //sensors

    public ShooterSubsystem(){
        //motors
        shooterMotor = new CANSparkMax(13, MotorType.kBrushless);
        shooterMotorTwo = new CANSparkMax(14, MotorType.kBrushless);

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
        shooterMotor.setVoltage(speed * 12);
        System.out.println("shooter motor speed is: " + shooterMotor.get());
    }

}