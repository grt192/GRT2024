package frc.robot.subsystems.elevator;

import java.util.EnumSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private NetworkTableInstance elevatorNetworkTableInstance;
    private NetworkTable elevatorNetworkTable;
    private NetworkTableEntry elevatorNetworkTablePositionEntry;

    private volatile boolean isManual = false;
    private double manualPower = 0;
    
    private ElevatorState state = ElevatorState.GROUND;
    private ElevatorState targetState = ElevatorState.GROUND;

    private final CANSparkMax extensionMotor;
    private RelativeEncoder extensionEncoder;
    private SparkPIDController extensionPidController;
    private final Timer timer = new Timer();

    //PID Values

    private final CANSparkMax extensionFollow;

    private final DigitalInput zeroLimitSwitch;

    //Controller for testing.

    public ElevatorSubsystem() {
        //Print out current position for debug & measurement
        //System.out.print(extensionEncoder.getPosition());

        timer.start();
        
        zeroLimitSwitch = new DigitalInput(ElevatorConstants.ZERO_LIMIT_ID); 
        elevatorNetworkTableInstance = NetworkTableInstance.getDefault();
        elevatorNetworkTable = elevatorNetworkTableInstance.getTable("elevator");
        elevatorNetworkTablePositionEntry = elevatorNetworkTable.getEntry("target_position");
        elevatorNetworkTable.addListener("target_position",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (NetworkTable table, String key, NetworkTableEvent event) -> {
            String message = event.valueData.value.getString();
            System.out.println(message);
            if(message.equals("GROUND")){
                System.out.println("Setting Target to Ground");
                this.setTargetState(ElevatorState.GROUND);
            }
            else if(message.equals("SPEAKER")){
                System.out.println("Setting Target to TRAP");
                this.setTargetState(ElevatorState.TRAP);
            }
            else if(message.equals("AMP")){
                System.out.println("Setting Target to AMP");
                this.setTargetState(ElevatorState.AMP);
            }
            else return;
            ElevatorState currentTargetState = this.getTargetState();
            if(currentTargetState.equals(ElevatorState.AMP)){
                System.out.println("New target state is AMP!");
            }
            else if(currentTargetState.equals(ElevatorState.GROUND)){
                System.out.println("New target state is GROUND!");
            }
            else if(currentTargetState.equals(ElevatorState.TRAP)){
                System.out.println("New target state is TRAP!");
            }
        });
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
        extensionPidController.setOutputRange(-0.7, 1);
    }
    @Override
    public void periodic(){
        //System.out.println(elevatorNetworkTablePositionEntry.getString("default"));
        // if(timer.advanceIfElapsed(.2)){
        //     System.out.println(zeroLimitSwitch.get());
        // }
        
        //System.out.println(this.getTargetState());
        if(isManual){
            //Add some factors for better control.
            extensionMotor.set(this.manualPower);
            return;
        }
         
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get()){
            // System.out.println("RESET LIMIT");
            extensionEncoder.setPosition(0); 
        }
        
        //extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //this through overun when no motor connected.
    }
    
    public boolean atState(ElevatorState state){
        double distance = Math.abs(this.getExtensionMeters() - state.getExtendDistanceMeters());
        if(distance < ElevatorConstants.EXTENSION_TOLERANCE){
            return true;
        }
        else{
            return false;
        }
    }

    public void setState(ElevatorState state) {
        this.state = state;
        return;
    }
    
    public void setTargetState(ElevatorState targetState){
        extensionPidController.setReference(targetState.getExtendDistanceMeters(), ControlType.kPosition, 0, 0.03, ArbFFUnits.kPercentOut);
        return;
    }

    public void setManual(){
        this.isManual = true;
        return;
    }

    public void setAuto(){
        this.isManual = false;
        return;
    }
    public double getExtensionMeters() { 
        return extensionEncoder.getPosition();
    }

    public ElevatorState getState() {
        return this.state;
    }

    public ElevatorState getTargetState(){
        return this.targetState;
    }
    
    public void setManualPower(double power){
        this.manualPower = power;
    } 
}