// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants;

import javax.imageio.plugins.tiff.TIFFDirectory;

import org.apache.commons.math3.ml.neuralnet.Network;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.util.TrackingTimer;

/** The subsystem that controls the rollers on the intake. */
public class IntakeRollerSubsystem extends SubsystemBase {
    private final CANSparkMax frontMotors;
    private final TalonSRX integrationMotor;
    private final DigitalInput frontSensor;
    private final DigitalInput rockwellSensor;
    private final DigitalInput ampSensor;
    // private ColorSensorV3 colorSensor;

    private boolean prevFrontSensorValue = false;

    private NetworkTableInstance ntInstance;
    private NetworkTable ntTable;
    private BooleanPublisher ntFrontPublisher;
    private BooleanPublisher ntBackPublisher;

    private NetworkTable motorsNTTable;
    private NetworkTableEntry frontMotorCurrentEntry;
    private NetworkTableEntry frontMotorVoltageEntry;
    private NetworkTableEntry frontMotorTemperatureEntry;
    private NetworkTableEntry integrationMotorCurrentEntry;
    private NetworkTableEntry integrationMotorVoltageEntry;
    private NetworkTableEntry integrationMotorTemperatureEntry;

    private NetworkTable intakeNTTable;
    private NetworkTableEntry frontSensorEntry;
    private NetworkTableEntry rockwellSensorEntry;
    private NetworkTableEntry ampSenSorEntry;

    private final LightBarSubsystem lightBarSubsystem;

    private Timer colorResetTimer;
    private TrackingTimer sensorTimer = new TrackingTimer();

    /** 
     * Subsystem controls the front, middle, and integration rollers for the intake.
     */
    public IntakeRollerSubsystem(LightBarSubsystem lightBarSubsystem) {
        integrationMotor = new TalonSRX(IntakeConstants.INTEGRATION_MOTOR_ID);
        frontMotors = new CANSparkMax(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
        frontMotors.setIdleMode(IdleMode.kBrake);
        frontMotors.setInverted(true);
        frontSensor = new DigitalInput(3);
        rockwellSensor = new DigitalInput(4);
        ampSensor = new DigitalInput(5);
    
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("RobotStatus");
        intakeNTTable = ntInstance.getTable("Intake");
        frontSensorEntry = intakeNTTable.getEntry("FrontSensor");
        rockwellSensorEntry = intakeNTTable.getEntry("Rockwell");
        ampSenSorEntry = intakeNTTable.getEntry("AMPSensor");
        ntFrontPublisher = ntTable.getBooleanTopic("FrontSensor").publish();
        ntBackPublisher = ntTable.getBooleanTopic("BackSensor").publish();

        motorsNTTable = ntInstance.getTable("Motors");
        frontMotorCurrentEntry = motorsNTTable.getEntry("Intake17Current");
        frontMotorVoltageEntry = motorsNTTable.getEntry("Intake17Voltage");
        frontMotorTemperatureEntry = motorsNTTable.getEntry("Intake17Temperature");
        integrationMotorCurrentEntry = motorsNTTable.getEntry("Intake19Current");
        integrationMotorVoltageEntry = motorsNTTable.getEntry("Intake19Voltage");
        integrationMotorTemperatureEntry = motorsNTTable.getEntry("Intake19Temperature");
        this.lightBarSubsystem = lightBarSubsystem;

        // colorResetTimer = new Timer();
        // colorResetTimer.start();
    }

    /**
     * Gets whether a note is at the front sensor currently.
     *
     * @return Whether a note is at the front sensor currently.
     */
    public boolean getFrontSensorReached() {

        if (prevFrontSensorValue && !getFrontSensorValue()) {
            sensorTimer.reset();
            sensorTimer.start();
        }

        return ((prevFrontSensorValue && !getFrontSensorValue()) || (!sensorTimer.hasElapsed(.2) && sensorTimer.hasStarted()));
    }

    /**
     * Gets the front sensor value.
     *
     * @return The current measurement of the front sensor.
     */
    public boolean getFrontSensorValue() {
        return !frontSensor.get();
    }

    /**
     * Gets the back sensor value.
     *
     * @return The current measurement of the back sensor.
     */
    public boolean getRockwellSensorValue() {
        return !rockwellSensor.get();
    }

    /**
     * sets the speed for the front and integration rollers.
     *
     * @param frontSpeed The speed for the front rollers.
     * @param integrationSpeed The speed for the integration rollers.
     */
    public void setRollSpeeds(double frontSpeed, double integrationSpeed) {
        frontMotors.set(frontSpeed);
        integrationMotor.set(TalonSRXControlMode.PercentOutput, integrationSpeed);
    }

    /** Gets motor output of the integration roller.
     *
     * @return The motor speed in percent.
     */
    public double getIntegrationSpeed() {
        return integrationMotor.getMotorOutputPercent();
    }

    public boolean getAmpSensor() {
        return ampSensor.get();
    }

    @Override
    public void periodic() {
        frontSensorEntry.setBoolean(getFrontSensorReached());
        rockwellSensorEntry.setBoolean(getRockwellSensorValue());
        ampSenSorEntry.setBoolean(getAmpSensor());

        frontMotorCurrentEntry.setDouble(frontMotors.getOutputCurrent());
        frontMotorVoltageEntry.setDouble(frontMotors.getBusVoltage());
        frontMotorTemperatureEntry.setDouble(frontMotors.getMotorTemperature());
        integrationMotorCurrentEntry.setDouble(integrationMotor.getSupplyCurrent());
        integrationMotorVoltageEntry.setDouble(integrationMotor.getMotorOutputVoltage());
        integrationMotorTemperatureEntry.setDouble(integrationMotor.getTemperature());

        ntFrontPublisher.set(getFrontSensorReached());
        prevFrontSensorValue = getFrontSensorValue();
        if (getFrontSensorValue()) {
            lightBarSubsystem.setLightBarStatus(LightBarStatus.HOLDING_NOTE, 2);
        }
    }
}
