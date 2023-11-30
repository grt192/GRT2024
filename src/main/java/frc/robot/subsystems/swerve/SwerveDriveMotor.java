package frc.robot.subsystems.swerve;

public interface  SwerveDriveMotor {
    
    public void setVelocity(double velocity);

    public void setPower(double power);

    public void configPID(double P, double I, double D, double FF);

    public double getDistance();

    public double getVelocity();
}
