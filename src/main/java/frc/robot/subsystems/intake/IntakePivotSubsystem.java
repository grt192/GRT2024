package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** The subsystem for the pivot on the intake. */
public class IntakePivotSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor;

    private PositionVoltage request = new PositionVoltage(0).withSlot(0);
    private VoltageConfigs voltageConfig = new VoltageConfigs();
    private double setPos = 0;
    private static final double FORWARD_VOLTAGE = 1.75;

    private NetworkTableInstance ntInstance;
    private NetworkTable motorsNTTable;
    private NetworkTableEntry intake16CurrentEntry;
    private NetworkTableEntry intake16VoltageEntry;
    private NetworkTableEntry intake16TemperatureEntry;
    
    /**
     * Subsystem for controlling the pivot on the intake.
     */
    public IntakePivotSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        // intakeencoder = new Encoder(1, 2);
        // extendedlimitswitch = new DigitalInput(extendedlimitswitchID);
        // retractedlimitswitch = new DigitalInput(retractedlimitswitchID);

        Slot0Configs slot0Configs = new Slot0Configs();

        slot0Configs.kP = IntakeConstants.PIVOT_P;
        slot0Configs.kI = IntakeConstants.PIVOT_I;
        slot0Configs.kD = IntakeConstants.PIVOT_D;

        voltageConfig.PeakForwardVoltage = FORWARD_VOLTAGE;
        voltageConfig.PeakReverseVoltage = -16;


        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.setPosition(0);
        pivotMotor.getConfigurator().apply(voltageConfig);
        

        request.withLimitForwardMotion(false);

        ntInstance = NetworkTableInstance.getDefault();
        motorsNTTable = ntInstance.getTable("Motors");
        intake16CurrentEntry = motorsNTTable.getEntry("Intake16Current");
        intake16VoltageEntry = motorsNTTable.getEntry("Intake16Voltage");
        intake16TemperatureEntry = motorsNTTable.getEntry("Intake16Temperature");
    }

    /**
     * Sets position of pivot to a double.
     *
     * @param position The position to set the pivot to. 1 is fully extended, 0 is fully stowed.
     */

    public void setPosition(double position) {
        pivotMotor.setControl(request.withPosition(position / IntakeConstants.PIVOT_CONVERSION_FACTOR));
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

    /**
     * Enables/disables the pivot motor's forward voltage limit for deploying the intake.
     * When enabled, the linkage will not slam down when deploying.
     *
     * @param enabled True enables the limit, false disables it.
     */
    public void enablePowerLimit(boolean enabled) {
        if (enabled) {
            voltageConfig.PeakForwardVoltage = FORWARD_VOLTAGE;
        } else {
            voltageConfig.PeakForwardVoltage = 16;
        }

        pivotMotor.getConfigurator().apply(voltageConfig);
    }

    @Override
    public void periodic() {
        pivotMotor.setControl(request.withPosition(setPos / IntakeConstants.PIVOT_CONVERSION_FACTOR));
  
        intake16CurrentEntry.setDouble(pivotMotor.getSupplyCurrent().getValueAsDouble());
        intake16VoltageEntry.setDouble(pivotMotor.getMotorVoltage().getValueAsDouble());
        intake16TemperatureEntry.setDouble(pivotMotor.getDeviceTemp().getValueAsDouble());
    }
}