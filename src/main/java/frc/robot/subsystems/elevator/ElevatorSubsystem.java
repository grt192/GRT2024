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
import edu.wpi.first.wpilibj2.command.CommandScheduler;


import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.ElevatorToLimitSwitchCommand;

/** Represents the elevator mechanism. */
public class ElevatorSubsystem extends SubsystemBase {
    // private NetworkTableInstance elevatorNetworkTableInstance;
    // private NetworkTable elevatorNetworkTable;

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
        extensionMotor.setInverted(true);
        extensionMotor.setClosedLoopRampRate(0.3);
        
        
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
        extensionPidController.setD(ElevatorConstants.EXTENSION_D);
        extensionPidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.EXTENSION_TOLERANCE, 0);
        extensionPidController.setOutputRange(-0.3, 1);
    }

    @Override 
    public void periodic() {
        CommandScheduler.getInstance().run();
 
        if (isManual) {
            //Add some factors for better control.
            extensionMotor.set(this.manualPower);
        }
         
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get() && ElevatorConstants.LIMIT_SWITCH_ENABLED) {
            //if limit switch tells us it's at the bottom
            extensionEncoder.setPosition(0); 
            this.setManualPower(0);//stops the motor in manual mode to avoid motor stall.
        }

        // if (ElevatorConstants.LIMIT_SWITCH_ENABLED &&
        //     zeroLimitSwitch != null && zeroLimitSwitch.get() &&
        //     Math.abs(this.getExtensionPercent()) < ElevatorConstants.EXTENSION_TOLERANCE) { 
        //         //if the encoder thinks the elevator is at ground
        //         CommandScheduler.getInstance().schedule(new ElevatorToLimitSwitchCommand(this));
        //         //slowly go down to limit switch
        // } 
    }
    
    /**
     * Returns if the elevator is touching the limit switch.
     *
     * @return true if the elevator is touching the limit switch, false if not.
     */
    public boolean atGround() {
        if (ElevatorConstants.LIMIT_SWITCH_ENABLED) {
            if (zeroLimitSwitch != null && zeroLimitSwitch.get()) {
                return true;
            } else {
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
        if (distance < ElevatorConstants.EXTENSION_TOLERANCE) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Set the target state to the state inputted.
     *
     * @param targetState the state you want to set to targetState.
     */
    public void setTargetState(ElevatorState targetState) {
        extensionPidController.setReference(
            targetState.getExtendDistanceMeters(), ControlType.kPosition, 0, 0.03, ArbFFUnits.kPercentOut
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
        if (zeroLimitSwitch != null) {
            return !zeroLimitSwitch.get();
        } else {
            throw new Error("Can't find limit switch!");
        }
    }

    /**
     * Zeros the encoder to position 0.
     *
     */
    public void zeroEncoder() {
        extensionEncoder.setPosition(0);
    }
}