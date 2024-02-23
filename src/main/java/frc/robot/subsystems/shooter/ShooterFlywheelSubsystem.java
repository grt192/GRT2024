// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Inits motors and state enums for shooter subsystem. */
public class ShooterFlywheelSubsystem extends SubsystemBase {

    int IDNUMBER = 10; //so I remember to change them later
    public final double SHOOTER_MOTOR_SPEED = 1;

    //motors
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBottom;

    //devices
    private ShooterState currentState; //an enum state thing

    /** Motors assigned. */
    public ShooterFlywheelSubsystem(){
        //motors
        shooterMotorTop = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ID);
        shooterMotorBottom = new TalonFX(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID);

        shooterMotorTop.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorBottom.setNeutralMode(NeutralModeValue.Coast);
        
        //enums
        setShooterState(ShooterState.VERTICAL);
    }

    /** Gets current state of shooter. */
    public ShooterState getShooterState(ShooterState state) {
        return currentState;
    }

    /** Sets current state of shooter. */
    public void setShooterState(ShooterState newState) {
        currentState = newState;
    }

    /** Sets shooting motor speed.  */
    public void setShooterMotorSpeed(double topSpeed, double bottomSpeed) {
        shooterMotorTop.setVoltage(-topSpeed * 12);
        shooterMotorBottom.setVoltage(-bottomSpeed * 12);

        //System.out.println("shooter motor speed is: " + shooterMotorTop.get());
    }

    /** Sets shooting motor speed for only one speed. */
    public void setShooterMotorSpeed(double speed) {
        shooterMotorBottom.setVoltage(-speed * 12);
        shooterMotorTop.setVoltage(speed * 12);
    }

}