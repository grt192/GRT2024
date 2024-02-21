// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//1 = clockwise, -1 = counterclockwise

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollersSubsystem extends SubsystemBase {
    private final TalonSRX frontMotors;
    private final TalonSRX integrationMotor;
    private final AnalogPotentiometer sensor;

    /** 
     * Subsystem controls the front, middle, and integration rollers for the intake
     */
    public IntakeRollersSubsystem() {
        integrationMotor = new TalonSRX(INTEGRATION_MOTOR_ID);
        frontMotors = new TalonSRX(FRONT_MOTOR_ID);
        frontMotors.setInverted(true);
        sensor = new AnalogPotentiometer(sensorID);
    }

    /**
     * if the sensor is reached (true) or not
     */
    public boolean sensorNow() {
        if (sensor.get() >= sensorreached) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * sets the speed for the front and middle rollers
     * @param frontspeed
     */
    public void setRollSpeed(double frontspeed) {
        frontMotors.set(TalonSRXControlMode.PercentOutput, frontspeed);
    }

    /**
     * sets the speed for the front, middle, and integration rollers
     * @param frontspeed
     * @param integrationspeed
     */
    public void setAllRollSpeed(double frontspeed, double integrationspeed) {
        frontMotors.set(TalonSRXControlMode.PercentOutput, frontspeed);
        integrationMotor.set(TalonSRXControlMode.PercentOutput, integrationspeed);
    }

    /**
     * Sets the rollers to go outwards
     * @param pressedA
     */
    public void setRollersOutwards(Boolean pressedA) {
        if (pressedA == true)
            frontMotors.set(TalonSRXControlMode.PercentOutput, rollersclockwise);

    }

    /**
     * sets the rollers to go inwards (intake)
     * @param pressedB
     */
    public void setRollersInwards(Boolean pressedB) {
        if (pressedB == true)
            frontMotors.set(TalonSRXControlMode.PercentOutput, rollersclockwise);

    }

    @Override
    public void periodic() {
        // System.out.println("intake distance sensor: " + sensor.get());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
