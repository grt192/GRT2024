package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.MotorUtil;

public class ClimbArm {
    private final CANSparkMax winchMotor;
    private final RelativeEncoder extensionEncoder;
    private final SparkPIDController extensionPIDController;

    private final DigitalInput zeroLimitSwitch;

    private double targetExtension = 0;

    public ClimbArm(int WINCH_MOTOR_ID, int ZERO_LIMIT_ID) {
        winchMotor = MotorUtil.createSparkMax(WINCH_MOTOR_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(true);
            sparkMax.setClosedLoopRampRate(EXTENSION_RAMP_RATE);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) RAISE_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) LOWER_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        });

        extensionEncoder = winchMotor.getEncoder();
        extensionEncoder.setPositionConversionFactor(AXLE_PERIMETER_METERS / WINCH_REDUCTION);
        extensionEncoder.setPosition(0);

        extensionPIDController = MotorUtil.createSparkPIDController(winchMotor, extensionEncoder);
        extensionPIDController.setP(EXTENSION_P);
        extensionPIDController.setI(EXTENSION_I);
        extensionPIDController.setD(EXTENSION_D);
        extensionPIDController.setSmartMotionAllowedClosedLoopError(EXTENSION_TOLERANCE_METERS, 0);

        zeroLimitSwitch = new DigitalInput(LEFT_ZERO_LIMIT_ID);
    }

    public void update() {
        if (zeroLimitSwitch != null && zeroLimitSwitch.get())
            resetEncoder();

        extensionPIDController.setReference(targetExtension, ControlType.kPosition);

    }
    
    public void setTargetExtension(double targetExtension) {
        this.targetExtension = MathUtil.clamp(targetExtension, LOWER_LIMIT_METERS, RAISE_LIMIT_METERS);
    }

    public double getTargetExtension() {
        return targetExtension;
    }

    public double getCurrentExtension() {
        return extensionEncoder.getPosition();
    }

    public boolean isAtTargetExtension() {
        return Math.abs(getCurrentExtension() - getTargetExtension()) < EXTENSION_TOLERANCE_METERS; 
    }

    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}
