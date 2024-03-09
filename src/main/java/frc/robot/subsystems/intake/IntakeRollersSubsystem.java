// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeRollersSubsystem extends SubsystemBase {
    private final CANSparkMax frontMotors;
    private final TalonSRX integrationMotor;
    private final AnalogPotentiometer frontSensor;
    private final AnalogPotentiometer backSensor;
    private final ColorSensorV3 colorSensor;

    private NetworkTableInstance ntInstance;
    private NetworkTable ntTable;
    private BooleanPublisher ntFrontPublisher;
    private BooleanPublisher ntBackPublisher;

    /** 
     * Subsystem controls the front, middle, and integration rollers for the intake.
     */
    public IntakeRollersSubsystem() {
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
    }

    /**
     * if the sensor is reached (true) or not
     */
    public boolean frontSensorNow() {
        if (frontSensor.get() >= IntakeConstants.FRONT_SENSOR_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    }

    public double getFrontSensor() {
        return frontSensor.get();
    }

    public boolean backSensorNow() {
        return backSensor.get() > IntakeConstants.BACK_SENSOR_THRESHOLD;
    }

    public double getBackSensor() {
        return backSensor.get();
    }

    
    /** Returns the color seen by the color sensor.
     *
     * @return the color seen by the color sensor.
     */
    public Color getColorSensor() {
        return colorSensor.getColor();
    }

    /** Return whether the color sensor detects a note or not.
     *
     * @return whether the color sensor detects a note or not.
     */
    public boolean getNoteColorDetected() { 
        return getColorSensor().red >= IntakeConstants.COLOR_SENSOR_RED_THRESHOLD;
    }

    /**
     * sets the speed for the front and middle rollers
     * @param frontspeed
     */
    public void setRollSpeed(double frontspeed) {
        frontMotors.set(frontspeed);
    }

    /**
     * sets the speed for the front, middle, and integration rollers
     * @param frontspeed
     * @param integrationspeed
     */
    public void setAllRollSpeed(double frontspeed, double integrationspeed) {
        frontMotors.set(frontspeed);
        integrationMotor.set(TalonSRXControlMode.PercentOutput, integrationspeed);
    }

    @Override
    public void periodic() {
        System.out.println("intake color sensor: " + getColorSensor().red);
        // This method will be called once per scheduler run
        ntFrontPublisher.set(frontSensorNow());
        ntBackPublisher.set(backSensorNow());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
