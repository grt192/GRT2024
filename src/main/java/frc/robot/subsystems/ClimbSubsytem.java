package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MotorUtil;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsytem extends SubsystemBase{
    private final CANSparkMax winchMotor;

    public ClimbSubsytem() {
        winchMotor = MotorUtil.createSparkMax()
    }

}
