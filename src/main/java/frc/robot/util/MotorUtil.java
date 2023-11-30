package frc.robot.util;

import java.util.function.Consumer;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public class MotorUtil {
    /**
     * Creates a WPI_TalonSRX on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the Talon.
     * @return The configured Talon.
     */
    public static WPI_TalonSRX createTalonSRX(int deviceId) {
        WPI_TalonSRX talon = new WPI_TalonSRX(deviceId);

        // Set 60.0 amp current limit to kick in after 0.2 seconds
        checkError(deviceId, talon.configFactoryDefault(), "factory reset");
        checkError(deviceId, talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60.0, 60.0, 0.2)), "current limit");

        return talon;
    }

    /**
     * Creates a WPI_TalonSRX on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the Talon.
     * @return The configured Talon.
     */
    public static WPI_TalonFX createTalonFX(int deviceId) {
        WPI_TalonFX talon = new WPI_TalonFX(deviceId);

        // Set 60.0 amp current limit to kick in after 0.2 seconds
        checkError(deviceId, talon.configFactoryDefault(), "factory reset");
        checkError(deviceId, talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60.0, 60.0, 0.2)), "current limit");

        return talon;
    }

    /**
     * Creates a CANSparkMax on a given device ID and motor type, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @param motorType The SparkMax's motor type (kBrushed or kBrushless).
     * @param configureMotor A callback to configure the motor further before settings are burned to flash.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId, MotorType motorType, Consumer<CANSparkMax> configureMotor) {
        CANSparkMax spark = new CANSparkMax(deviceId, motorType);

        // Set 60.0 amp current limit
        checkError(deviceId, spark.restoreFactoryDefaults(), "factory reset");
        checkError(deviceId, spark.setSmartCurrentLimit(60), "current limit");

        // Apply manually configured settings
        configureMotor.accept(spark);
        // checkError(deviceId, spark.burnFlash(), "burn flash");

        return spark;
    }

    /**
     * Creates a brushless CANSparkMax on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @param configureMotor A callback to configure the motor further before settings are burned to flash.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId, Consumer<CANSparkMax> configureMotor) {
        return createSparkMax(deviceId, MotorType.kBrushless, configureMotor);
    }

    /**
     * Creates a brushless CANSparkMax on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax(int deviceId) {
        return createSparkMax(deviceId, MotorType.kBrushless, (sparkMax) -> {});
    }

    /**
     * Creates a brushless CANSparkMax for a NEO 550 on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @param configureMotor A callback to configure the motor further before settings are burned to flash.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax550(int deviceId, Consumer<CANSparkMax> configureMotor) {
        CANSparkMax spark = new CANSparkMax(deviceId, MotorType.kBrushless);

        // Set 20.0 amp current limit
        checkError(deviceId, spark.restoreFactoryDefaults(), "factory reset");
        checkError(deviceId, spark.setSmartCurrentLimit(20), "current limit");

        // Apply manually configured settings
        configureMotor.accept(spark);
        // checkError(deviceId, spark.burnFlash(), "burn flash");

        return spark;
    }

    /**
     * Creates a brushless CANSparkMax for a NEO 550 on a given device ID, configuring it with global defaults.
     * @param deviceId The CAN ID of the SparkMax.
     * @return The configured SparkMax.
     */
    public static CANSparkMax createSparkMax550(int deviceId) {
        return createSparkMax550(deviceId, (sparkMax) -> {});
    }

    /**
     * Creates a SparkMaxPIDController from a given SparkMax and feedback device.
     * @param spark The SparkMax to get a PID controller for.
     * @param encoder The feedback device to use for PID.
     * @return The configured SparkMaxPIDController.
     */
    public static SparkMaxPIDController createSparkMaxPIDController(CANSparkMax spark, MotorFeedbackSensor encoder) {
        SparkMaxPIDController pidController = spark.getPIDController();

        // Set feedback device
        checkError(spark.getDeviceId(), pidController.setFeedbackDevice(encoder), "PID feedback device");

        return pidController;
    }

    /**
     * Checks a CANSparkMax configuration call for an error, reporting it if it exists.
     * @param id The CAN ID of the SparkMax.
     * @param error The error returned by the configuration.
     * @param field The field being configured.
     */
    private static void checkError(int id, REVLibError error, String field) {
        if (error == REVLibError.kOk) return;
        DriverStation.reportError("Error configuring [" + field + "] on SparkMax " + id + ": " + error.name(), false);
    }

    /**
     * Checks a TalonSRX/FX configuration call for an error, reporting it if it exists.
     * @param id The CAN ID of the Talon.
     * @param error The error returned by the configuration.
     * @param field The field being configured.
     */
    private static void checkError(int id, ErrorCode error, String field) {
        if (error == ErrorCode.OK) return;
        DriverStation.reportError("Error configuring [" + field + "] on Talon " + id + ": " + error.name(), false);
    }
}
