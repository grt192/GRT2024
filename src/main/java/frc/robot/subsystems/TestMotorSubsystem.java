package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private double motorSpeed;
    private final DigitalInput limit;

    public TestMotorSubsystem(int motorPort, boolean inverted, int limitPort) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        motor.enableSoftLimit(SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        limit = new DigitalInput(limitPort);
    }

    @Override
    public void periodic() {
        motor.set(motorSpeed);
        // System.out.println(this.toString() + " current:" + motor.getOutputCurrent());
        //System.out.println(limit.get());
    }

    public void setMotorSpeed(double motorSpeed) {
        this.motorSpeed = MathUtil.clamp(motorSpeed, -1.0, +1.0);
    }
}