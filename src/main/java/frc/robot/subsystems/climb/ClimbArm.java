package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.MotorUtil;

public class ClimbArm {
    private final CANSparkMax winchMotor;
    private RelativeEncoder extensionEncoder;

    private final DigitalInput zeroLimitSwitch;

    private double desiredExtension = 0;

    public ClimbArm(int WINCH_MOTOR_ID, int ZERO_LIMIT_ID) {
        winchMotor = MotorUtil.createSparkMax(WINCH_MOTOR_ID, (sparkMax) -> {
            sparkMax.setIdleMode(IdleMode.kBrake); 
            sparkMax.setInverted(false);

            extensionEncoder = sparkMax.getEncoder();
            extensionEncoder.setPositionConversionFactor(AXLE_PERIMETER_METERS / WINCH_REDUCTION);
            extensionEncoder.setPosition(0);

            sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) EXTENSION_LIMIT_METERS);
            sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
            sparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
            sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
        });

        zeroLimitSwitch = new DigitalInput(LEFT_ZERO_LIMIT_ID);
    }

    public void update() {
        if (zeroLimitSwitch != null && zeroLimitSwitch.get())
            resetEncoder();

        if (extensionEncoder.getPosition() == 0 && !zeroLimitSwitch.get())
            System.out.println(this.toString() + "encoder uncalibrated!");

        if (isAtExtension())
            winchMotor.set(0);
        else
            winchMotor.set(desiredExtension > extensionEncoder.getPosition()
                ? MAX_WINCH_POWER : -MAX_WINCH_POWER);

    }
    
    public void goToExtension(double desiredHeight) {
        desiredExtension = MathUtil.clamp(desiredHeight, 0, EXTENSION_LIMIT_METERS);
    }

    public boolean isAtExtension() {
        return Math.abs(extensionEncoder.getPosition() - desiredExtension) < EXTENSION_TOLERANCE_METERS; 
    }

    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}
