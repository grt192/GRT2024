package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import java.util.EnumSet;


public class ElevatorSubsystem extends SubsystemBase {
    private NetworkTableInstance elevatorNetworkTableInstance;
    private NetworkTable elevatorNetworkTable;

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
        //Print out current position for debug & measurement
        //System.out.print(extensionEncoder.getPosition());

        timer.start();
        
        zeroLimitSwitch = new DigitalInput(ElevatorConstants.ZERO_LIMIT_ID); 
        elevatorNetworkTableInstance = NetworkTableInstance.getDefault();
        elevatorNetworkTable = elevatorNetworkTableInstance.getTable("elevator");
        elevatorNetworkTable.addListener("target_position",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (NetworkTable table, String key, NetworkTableEvent event) -> {
                String message = event.valueData.value.getString();
                System.out.println(message);
                if (message.equals("GROUND")) {
                    System.out.println("Setting Target to Ground");
                    this.setTargetState(ElevatorState.ZERO);
                } else if (message.equals("SPEAKER")) {
                    System.out.println("Setting Target to TRAP");
                    this.setTargetState(ElevatorState.TRAP);
                } else if (message.equals("AMP")) {
                    System.out.println("Setting Target to AMP");
                    this.setTargetState(ElevatorState.AMP);
                } else {
                    return;
                }
                ElevatorState currentTargetState = this.getTargetState();
                if (currentTargetState.equals(ElevatorState.AMP)) {
                    System.out.println("New target state is AMP!");
                } else if (currentTargetState.equals(ElevatorState.ZERO)) {
                    System.out.println("New target state is GROUND!");
                } else if (currentTargetState.equals(ElevatorState.TRAP)) {
                    System.out.println("New target state is TRAP!");
                }
            }
        );
        //this entry is working!
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
        extensionFollow.setInverted(true);

        extensionPidController = extensionMotor.getPIDController();
        extensionPidController.setP(ElevatorConstants.EXTENSION_P);
        extensionPidController.setI(ElevatorConstants.EXTENSION_I);
        extensionPidController.setD(ElevatorConstants.EXTENSION_D);
        extensionPidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.EXTENSION_TOLERANCE, 0);
        extensionPidController.setOutputRange(-0.3, 1);
    }

    @Override 
    public void periodic() {
        //System.out.println(elevatorNetworkTablePositionEntry.getString("default"));
        // if(timer.advanceIfElapsed(.2)){
        //     System.out.println(zeroLimitSwitch.get());
        // }

        
        // System.out.println(extensionEncoder.getPosition()); 
        
        //System.out.println(this.getTargetState());
        if (isManual) {
            //Add some factors for better control.
            extensionMotor.set(this.manualPower);
        }
         
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get()) {
            // System.out.println("RESET LIMIT");
            extensionEncoder.setPosition(0); 
        }
        
        //extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //this through overun when no motor connected.
    }
    
    /**
     * If the elevator is touching the limit switch.

     * @return true if the elevator is touching the limit switch, false if not.
     */
    public boolean atGround() {
        if (zeroLimitSwitch != null && zeroLimitSwitch.get()) {
            return true;
        } else {
            return false;
        }
    }
    /**
     * Check if the elevator is at a certain state.

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
     * Manually set the elevator's current state to the state inputted.

     * @param state the state you want to set to
     */
    public void setState(ElevatorState state) {
        this.state = state;
        return;
    }
    
    /**
     * set the target state to the state inputted.

     * @param targetState the state you want to set to targetState.
     */
    public void setTargetState(ElevatorState targetState) {
        extensionPidController.setReference(
            targetState.getExtendDistanceMeters(), ControlType.kPosition, 0, 0.03, ArbFFUnits.kPercentOut
        );
        return;
    }

    /**
     * set the mode to manual mode.
     */
    public void setManual() {
        this.isManual = true;
        return;
    }

    /**
     * set the mode to auto.
     */
    public void setAuto() {
        this.isManual = false;
        return;
    }
    
    /**
     * get the current position from the encoder in meters.

     * @return position in doubles.
     */
    public double getExtensionPercent() { 
        return extensionEncoder.getPosition();
    }

    /**
     * get the current state of the elevator.

     * @return the current state of the elevator
     */
    public ElevatorState getState() {
        return this.state;
    }

    /**
     * get current target state.

     * @return current target state.
     */
    public ElevatorState getTargetState() {
        return this.targetState;
    }
    
    /**
     * set the power in manual mode to.

     * @param power the power you want to set to
     */
    public void setManualPower(double power) {
        this.manualPower = power;
    } 
}