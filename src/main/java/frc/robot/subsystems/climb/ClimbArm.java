package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.MotorUtil;

public class ClimbArm {
    private final CANSparkMax winchMotor;
    private RelativeEncoder extensionEncoder;

    private final DigitalInput zeroLimitSwitch;

    private final Consumer<CANSparkMax> configureMotor = sparkMax -> {
        sparkMax.setIdleMode(IdleMode.kBrake); 
        sparkMax.setInverted(false);

        extensionEncoder = sparkMax.getEncoder();
        extensionEncoder.setPositionConversionFactor(Units.inchesToMeters(AXLE_RADIUS_INCHES) * 2 * Math.PI / WINCH_REDUCTION);
        extensionEncoder.setPosition(0);

        sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) Units.inchesToMeters(EXTENSION_LIMIT_INCHES));
        sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        sparkMax.setSoftLimit(SoftLimitDirection.kReverse, 0);
        sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    };

    private double desiredExtension = 0;

    public ClimbArm(int WINCH_MOTOR_ID, int ZERO_LIMIT_ID) {
        winchMotor = MotorUtil.createSparkMax(WINCH_MOTOR_ID, configureMotor);

        zeroLimitSwitch = new DigitalInput(LEFT_ZERO_LIMIT_ID);
    }

    public void goToExtension(double desiredHeight) {
        desiredExtension = MathUtil.clamp(0, desiredHeight, EXTENSION_LIMIT_METERS);
    }

    public void periodic() {
        if (zeroLimitSwitch != null && !zeroLimitSwitch.get())
            resetEncoder();

        if (extensionEncoder.getPosition() == 0 && zeroLimitSwitch.get())
            System.out.println(this.toString() + "encoder uncalibrated!");

        if (extensionEncoder.getPosition() == desiredExtension)
            winchMotor.set(0);
        else
            winchMotor.set(desiredExtension - extensionEncoder.getPosition());

    }

    private void resetEncoder() {
        extensionEncoder.setPosition(0);
    }
}
