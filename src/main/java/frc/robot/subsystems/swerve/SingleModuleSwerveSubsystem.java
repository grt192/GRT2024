package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class SingleModuleSwerveSubsystem extends BaseSwerveSubsystem{

    private static final double MAX_VEL = 5; // m/s STUB

    private Timer crimor;
    private boolean toRun;
    
    SwerveModule module;

    private double angle = 0;

    public SingleModuleSwerveSubsystem(SwerveModule module){
        toRun = false;
        this.module = module;
        crimor = new Timer();
        crimor.start();
    }

    public void setRawPowers(double drivePower, double steerPower){
        module.setRawPowers(drivePower, steerPower);
    }

    public void setRawPowersWithAngle(double drivePower, double angleRads){
        module.setRawPowersWithAngle(drivePower, angleRads);
    }

    public void setDrivePowers(double xPower, double yPower){

        double velocity = MAX_VEL * Math.sqrt(yPower * yPower + xPower * xPower) / Math.sqrt(2);
        if(Math.abs(xPower) < .01 && Math.abs(yPower) < .01 ){
            System.out.println(Math.atan2(0, 0));
            module.setRawPowers(0, 0);
            return;
        }
        double angle = Math.atan2(yPower, xPower);

        module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
    }

    public void setDrivePowers(double x, double y, double drivePower){
        if(!(x == 0 && y == 0)){
            angle = Math.atan2(y, x);
        }

        double velocity = MAX_VEL * drivePower;

        if(toRun){
            module.setDesiredState(new SwerveModuleState(velocity, new Rotation2d(angle)));
            // module.setRawPowersWithAngle(drivePower, angle);
        }

        // if (crimor.advanceIfElapsed(.1)){
        //     System.out.print(" set drive power" + twoDecimals(drivePower));
        //     System.out.print(" to run " + toRun);
        //     System.out.print(" current " + twoDecimals(module.getWrappedAngle().getDegrees()));
        //     System.out.println(" target " + twoDecimals(Math.toDegrees(MathUtil.angleModulus(angle))));
            
        // }
    }

    public double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

    public void toggletoRun(){
        toRun = !toRun;
        System.out.println(toRun);
    }


}
