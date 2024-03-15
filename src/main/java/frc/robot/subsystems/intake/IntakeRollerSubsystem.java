// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem that controls the rollers on the intake. */
public class IntakeRollerSubsystem extends SubsystemBase {
    private final CANSparkMax frontMotors;
    private final TalonSRX integrationMotor;
    private final AnalogPotentiometer frontSensor;
    private final AnalogPotentiometer backSensor;
    private ColorSensorV3 colorSensor;

    private NetworkTableInstance ntInstance;
    private NetworkTable ntTable;
    private BooleanPublisher ntFrontPublisher;
    private BooleanPublisher ntBackPublisher;

    private Timer colorResetTimer;

    /** 
     * Subsystem controls the front, middle, and integration rollers for the intake.
     */
    public IntakeRollerSubsystem() {
        integrationMotor = new TalonSRX(IntakeConstants.INTEGRATION_MOTOR_ID);
        frontMotors = new CANSparkMax(IntakeConstants.FRONT_MOTOR_ID, MotorType.kBrushless);
        frontMotors.setInverted(true);
        frontSensor = new AnalogPotentiometer(IntakeConstants.FRONT_SENSOR_ID);
        backSensor = new AnalogPotentiometer(IntakeConstants.BACK_SENSOR_ID);
        colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("RobotStatus");
        ntFrontPublisher = ntTable.getBooleanTopic("frontSensor").publish();
        ntBackPublisher = ntTable.getBooleanTopic("backSensor").publish();

        colorResetTimer = new Timer();
        colorResetTimer.start();
    }

    /**
     * Gets whether a note is at the front sensor currently.
     *
     * @return Whether a note is at the front sensor currently.
     */
    public boolean getFrontSensorReached() {
        if (frontSensor.get() >= IntakeConstants.FRONT_SENSOR_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets the front sensor value.
     *
     * @return The current measurement of the front sensor.
     */
    public double getFrontSensorValue() {
        return frontSensor.get();
    }

    /**
     * Gets whether a note is at the back sensor currently.
     *
     * @return Whether a note is at the back sensor currently.
     */
    public boolean getBackSensorReached() {
        return backSensor.get() > IntakeConstants.BACK_SENSOR_THRESHOLD;
    }

    /**
     * Gets the back sensor value.
     *
     * @return The current measurement of the back sensor.
     */
    public double getBackSensorValue() {
        return backSensor.get();
    }

    
    /** Returns the color seen by the color sensor.
     *
     * @return the color seen by the color sensor.
     */
    public Color getColorSensor() {
        return colorSensor.getColor();
        
    }

    /** Returns the red value seen by the color sensor.
     *
     * @return the red value seen by the color sensor.
     */
    public double getColorSensorRed() {
        return colorSensor.getRed();
    }

    /** Return whether the color sensor detects a note or not.
     *
     * @return whether the color sensor detects a note or not.
     */
    public boolean getNoteColorDetected() { 
        return colorSensor.getRed() >= IntakeConstants.COLOR_SENSOR_RED_THRESHOLD;
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

    @Override
    public void periodic() {
        ntFrontPublisher.set(getFrontSensorReached());
        ntBackPublisher.set(getBackSensorReached());
        if (colorSensor.getRed() == 0 || !colorSensor.isConnected() && colorResetTimer.advanceIfElapsed(2)) {
            System.out.println("Attempting to reset the color sensor.");
            colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        }
    }
}
