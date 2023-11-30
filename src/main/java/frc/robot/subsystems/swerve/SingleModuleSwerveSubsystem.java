package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleModuleSwerveSubsystem extends SubsystemBase{
    
    SwerveModule module;

    public SingleModuleSwerveSubsystem(SwerveModule module){
        this.module = module;
    }

    public void setRawPowers(double drivePower, double steerPower){
        module.setRawPowers(drivePower, steerPower);
    }

    public void setRawPowersWithAngle(double drivePower, double angleRads){
        module.setRawPowersWithAngle(drivePower, angleRads);
    }
}
