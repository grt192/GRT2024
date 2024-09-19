package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


import frc.robot.Constants.ElevatorConstants;

/** Represents the elevator mechanism. */
public class ElevatorSubsystem extends SubsystemBase {
    private NetworkTableInstance elevatorNetworkTableInstance;
    private NetworkTable elevatorNetworkTable;
    private NetworkTable motorsNetworkTable;
    private NetworkTableEntry limitSwitchEntry;
    private NetworkTableEntry targetStateEntry;
    private NetworkTableEntry extensionPercentEntry;
    private NetworkTableEntry motor10CurrentEntry;
    private NetworkTableEntry motor10VoltageEntry;
    private NetworkTableEntry motor10TemperatureEntry;
    private NetworkTableEntry motor11CurrentEntry;
    private NetworkTableEntry motor11VoltageEntry;
    private NetworkTableEntry motor11TemperatureEntry;

    private volatile boolean isManual = false;
    private double manualPower = 0;
    
    private ElevatorState state = ElevatorState.ZERO;
    private ElevatorState targetState = ElevatorState.ZERO;

    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkPIDController extensionPidController;
    private final Timer timer = new Timer();

    //PID Values

    private final CANSparkMax extensionFollow;

    private final DigitalInput zeroLimitSwitch;

    /**
     * Initializes elevator subsystem.
     */
    public ElevatorSubsystem() {

        timer.start();
        
        zeroLimitSwitch = new DigitalInput(ElevatorConstants.ZERO_LIMIT_ID); 

        extensionMotor = new CANSparkMax(ElevatorConstants.EXTENSION_ID, MotorType.kBrushless);
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setInverted(false);
        // extensionMotor.setClosedLoopRampRate(0.3);
        
        
        extensionEncoder = extensionMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        extensionEncoder.setVelocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR);
        extensionEncoder.setPosition(0);
        
        extensionFollow = new CANSparkMax(ElevatorConstants.EXTENSION_FOLLOW_ID, MotorType.kBrushless);
        extensionFollow.follow(extensionMotor);
        extensionFollow.setIdleMode(IdleMode.kBrake);
        extensionFollow.setInverted(false);

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(ElevatorConstants.EXTENSION_P);
        extensionPidController.setI(ElevatorConstants.EXTENSION_I);
        extensionPidController.setIZone(0.15);
        extensionPidController.setD(ElevatorConstants.EXTENSION_D);
        extensionPidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.EXTENSION_TOLERANCE, 0);
        extensionPidController.setOutputRange(-0.5, 1);

        elevatorNetworkTableInstance = NetworkTableInstance.getDefault();
        elevatorNetworkTable  = elevatorNetworkTableInstance.getTable("Elevator");
        motorsNetworkTable = elevatorNetworkTableInstance.getTable("Motors");
        limitSwitchEntry = elevatorNetworkTable.getEntry("LimitSwitch");
        extensionPercentEntry = elevatorNetworkTable.getEntry("ExtensionPercent");
        targetStateEntry = elevatorNetworkTable.getEntry("TargetState");
        motor10CurrentEntry = motorsNetworkTable.getEntry("Elevator10Current");
        motor10VoltageEntry = motorsNetworkTable.getEntry("Elevator10Voltage");
        motor10TemperatureEntry = motorsNetworkTable.getEntry("Elevator10Temperature");
        motor11CurrentEntry = motorsNetworkTable.getEntry("Elevator11Current");
        motor11VoltageEntry = motorsNetworkTable.getEntry("Elevator11Voltage");
        motor11TemperatureEntry = motorsNetworkTable.getEntry("Elevator11Temperature");

    }

    @Override 
    public void periodic() {
        //update elevator status to netowrk tables.
        extensionPercentEntry.setDouble(getExtensionPercent());
        limitSwitchEntry.setBoolean(getLimitSwitch());
        targetStateEntry.setString(getTargetState().toString()); 
        motor10CurrentEntry.setDouble(extensionMotor.getOutputCurrent());
        motor10VoltageEntry.setDouble(extensionMotor.getBusVoltage());
        motor10TemperatureEntry.setDouble(extensionMotor.getMotorTemperature());
        motor11CurrentEntry.setDouble(extensionFollow.getOutputCurrent());
        motor11VoltageEntry.setDouble(extensionFollow.getBusVoltage());
        motor11TemperatureEntry.setDouble(extensionFollow.getMotorTemperature());

        if (isManual) {
            //Add some factors for better control.
            extensionMotor.set(this.manualPower);
        }
         
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get() && ElevatorConstants.LIMIT_SWITCH_ENABLED) {
            //if limit switch tells us it's at the bottom
            extensionEncoder.setPosition(0); 
            this.setManualPower(0);//stops the motor in manual mode to avoid motor stall.
            extensionPidController.setIAccum(0);//Set Integral to 0 to prevent PID from moving further down.
        }
    }
    
    /**
     * Returns if the elevator is touching the limit switch.
     *
     * @return true if the elevator is touching the limit switch, false if not.
     */
    public boolean atGround() {
        if (ElevatorConstants.LIMIT_SWITCH_ENABLED && zeroLimitSwitch != null) {
            try{
                return zeroLimitSwitch.get();
            } catch (Exception e) {
                System.out.println(e);
                return false;
            }
        } else {
            return Math.abs(this.getExtensionPercent()) < ElevatorConstants.EXTENSION_TOLERANCE;
        }
    }

    /**
     * Checks if the elevator is at a certain state.
     *
     * @param state if the elevator is at this state.
     * @return boolean (if the elevator is at this state)
     */
    public boolean atState(ElevatorState state) {
        double distance = Math.abs(this.getExtensionPercent() - state.getExtendDistanceMeters());
        return distance < ElevatorConstants.EXTENSION_TOLERANCE;
    }

    public boolean atTrapState(){
        double distance = Math.abs(this.getExtensionPercent() - ElevatorState.TRAP.getExtendDistanceMeters());
        return distance < .03;
    }

    /**
     * Set the target state to the state inputted.
     *
     * @param targetState the state you want to set to targetState.
     */
    public void setTargetState(ElevatorState targetState) {
        extensionPidController.setReference(
            targetState.getExtendDistanceMeters(), ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut
        );
        this.targetState = targetState;
        return;
    }

    /**
     * Sets the mode to manual mode.
     */
    public void setManual() {
        this.isManual = true;
        return;
    }

    /**
     * Sets the mode to automatic.
     */
    public void setAuto() {
        this.isManual = false;
        return;
    }
    
    /**
     * Gets the current position from the encoder in meters.
     *
     * @return position in doubles.
     */
    public double getExtensionPercent() { 
        return extensionEncoder.getPosition();
    }

    /**
     * Gets the current state of the elevator.
     *
     * @return the current state of the elevator
     */
    public ElevatorState getState() {
        return this.state;
    }

    /**
     * Gets current target state.
     *
     * @return current target state.
     */
    public ElevatorState getTargetState() {
        return this.targetState;
    }
    
    /**
     * Sets the power in manual mode to.
     *
     * @param power the power you want to set to
     */
    public void setManualPower(double power) {
        this.manualPower = power;
    } 
    /**
     * Sets the motor power.
     *
     * @param power the power you want to set to
     */

    public void setMotorPower(double power) {
        extensionMotor.set(power);
    }
    /**
     * Returns the state of the limit switch, 0 is not triggered, 1 is triggered.
     *
     * @return false is not triggered, true is triggered. 
     */

    public boolean getLimitSwitch() {
            return !zeroLimitSwitch.get();
    }

    /**
     * Zeros the encoder to position 0.
     *
     */
    public void zeroEncoder() {
        extensionEncoder.setPosition(0);
    }

    public void setCoast(){
        extensionMotor.setIdleMode(IdleMode.kCoast);
        extensionFollow.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake(){
        extensionMotor.setIdleMode(IdleMode.kCoast);
        extensionFollow.setIdleMode(IdleMode.kCoast);
    }
}