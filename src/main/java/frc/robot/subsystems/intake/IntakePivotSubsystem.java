package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
    private final TalonFX pivotMotor;
    private final DigitalInput extendedlimitswitch;
    private final DigitalInput retractedlimitswitch;
    private final Encoder intakeencoder;
    private PositionVoltage request = new PositionVoltage(0).withSlot(0);
    private final double P = 0;
    private final double I = 0;
    private final double D = 0;
    private final double CONVERSION_FACTOR = 1; // TODO tune

    
     /**
      * Subsystem for controlling the pivot on the intake.
      */
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
        intakeencoder = new Encoder(1, 2);
        extendedlimitswitch = new DigitalInput(extendedlimitswitchID);
        retractedlimitswitch = new DigitalInput(retractedlimitswitchID);
        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;

        pivotMotor.getConfigurator().apply(slot0Configs);
    }

    /**
     * Sets position of pivot to a double.
     * @param position 
     */

    public void setPosition(double position) {
        pivotMotor.setControl(request.withPosition(position * CONVERSION_FACTOR));

    }

    /**
     * resets encoder to zero.
     */
    public void resetEncoder() {
        intakeencoder.reset();
    }

    /**
     * returns the encoder position.
     */
    public double encoderPosition() {
        return intakeencoder.get();
    }

    /**
     * sets the pivot motor to a speed
     * @param speed
     */
    public void movePivot(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * If pivot is extended (true) or not
     * @return if the pivot is extended. 
     */
    public boolean pivotisextended() {
        if (extendedlimitswitch.get()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * returns if pivot is retracted (true) or not
     */
    public boolean pivotisretracted() {
        if (retractedlimitswitch.get()) {
            return true;
        } else {
            return false;
        }
    }
}