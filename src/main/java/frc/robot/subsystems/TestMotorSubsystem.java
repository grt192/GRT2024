package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private double motorSpeed;

    public TestMotorSubsystem(int motorPort) {
        motor = new CANSparkMax(motorPort, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        motor.set(motorSpeed);
        System.out.println(motorSpeed);
    }

    public void setMotorSpeed(double motorSpeed) {
        this.motorSpeed = MathUtil.clamp(motorSpeed, -1.0, +1.0);
    }
}
