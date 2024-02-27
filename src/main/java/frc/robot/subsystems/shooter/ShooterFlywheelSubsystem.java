// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Inits motors and state enums for shooter subsystem. */
public class ShooterFlywheelSubsystem extends SubsystemBase {

    //motors
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBottom;

    //devices
    private ShooterState currentState; //an enum state thing
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    /** Motors assigned. */
    public ShooterFlywheelSubsystem(){
        //motors
        shooterMotorTop = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ID);
        shooterMotorBottom = new TalonFX(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID);

        shooterMotorTop.setInverted(true);
        shooterMotorBottom.setInverted(true);

        shooterMotorTop.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorBottom.setNeutralMode(NeutralModeValue.Coast);

        Slot0Configs configs = new Slot0Configs();

        configs.kP = 1;
        configs.kI = 0;
        configs.kD = 0;
        configs.kV = .12;
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
        double targetTopRPS = ShooterConstants.MAX_FLYWHEEL_RPS * topSpeed;
        double targetBottomRPS = ShooterConstants.MAX_FLYWHEEL_RPS * bottomSpeed;

        shooterMotorTop.setControl(request.withVelocity(targetTopRPS));
        shooterMotorBottom.setControl(request.withVelocity(targetBottomRPS));

        //System.out.println("shooter motor speed is: " + shooterMotorTop.get());
    }

    /** Sets shooting motor speed for only one speed. */
    public void setShooterMotorSpeed(double speed) {
        setShooterMotorSpeed(speed, speed);
    }

    public boolean atSpeed() {
        return shooterMotorTop.getClosedLoopError().getValueAsDouble() < 10.0
            && shooterMotorBottom.getClosedLoopError().getValueAsDouble() < 10.0;
    }

}