package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** The subsystem for the pivot on the intake. */
public class IntakePivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor;

    private double setPos = 0;
    private static final double FORWARD_VOLTAGE = 1.75;
    
    /**
     * Subsystem for controlling the pivot on the intake.
     */
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = IntakeConstants.PIVOT_P;
        slot0Configs.kI = IntakeConstants.PIVOT_I;
        slot0Configs.kD = IntakeConstants.PIVOT_D;

        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.setPosition(0);
    }

    /**
     * Sets position of pivot to a double.
     *
     * @param position The position to set the pivot to. 1 is fully extended, 0 is fully stowed.
     */

    public void setPosition(double position) {
        setPos = position;
    }

    /**
     * Resets encoder to zero.
     */
    public void resetEncoder() {
        pivotMotor.setPosition(0);
    }

    /**
     * Returns the encoder position.
     */
    public double getEncoderPosition() {
        return pivotMotor.getPosition().getValueAsDouble() * IntakeConstants.PIVOT_CONVERSION_FACTOR;
    }

    /** Returns whether or not the intake is at (or within tolerance of) its setpoint. */
    public boolean atPosition() {
        return Math.abs(getEncoderPosition() - setPos) < .05;
    }

    /**
     * Sets the pivot motor to a speed.
     *
     * @param speed The raw motor speed to set the pivot to.
     */
    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }


    @Override
    public void periodic() {

    }
}