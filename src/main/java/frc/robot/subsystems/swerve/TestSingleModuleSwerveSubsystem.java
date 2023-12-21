package frc.robot.subsystems.swerve;

import static frc.robot.Constants.TestSingleModuleSwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class TestSingleModuleSwerveSubsystem extends SingleModuleSwerveSubsystem {

    
    public static final double STEER_POWER = .06;
    public static final double DRIVE_POWER = .4;

    private int testCase;
    private boolean toRun;

    private double drive;
    private double steer;

    private Timer crimer;
    private double crime = 0;

    private Timer crimor;

    public TestSingleModuleSwerveSubsystem(SwerveModule module){
        super(module);

        testCase = 9;
        toRun = false;

        steer = 0;
        drive = 0;

        crimor = new Timer();
        crimor.start();
        crimer = new Timer();
        crimer.start();
    }
  

    @Override
    public void periodic() {
        // if (crimor.advanceIfElapsed(.1)){
        //     System.out.print("test case: " + testCase);
        //     // System.out.print(" error " + twoDecimals(module.getError()));
        //     System.out.println(" target " + twoDecimals(Math.toDegrees(MathUtil.angleModulus(steer))));
        // }

        if (!toRun) {
            super.setRawPowers(0, 0);
            return;
        }

        // These are the different test cases as requested by Alex
        switch(testCase) {
            
            case 0:
                //WEAR IN GEARS
                if(crimer.advanceIfElapsed(crime)){
                    drive = DRIVE_POWER * (Math.floor(Math.random() * 2) * 2 - 1); //Either 1 or -1
                    steer = STEER_POWER * (Math.floor(Math.random() * 2) * 2 - 1);
                    
                    crime = Math.random() * 20 + 20;
                }
                break;
            case 1:
                //DRIVE FORWARD
                drive = DRIVE_POWER;
                steer = 0;
                break;
            case 2:
                //DRIVE BACKWARD
                drive = -DRIVE_POWER;
                steer = 0;
                break;
            case 3:
                //STEER LEFT
                drive = 0;
                steer = STEER_POWER;
                break;
            case 4:
                //STEER RIGHT
                drive = 0;
                steer = -STEER_POWER;
                break;
            case 5:
                //FORWARD LEFT
                drive = DRIVE_POWER;
                steer = STEER_POWER;
                break;
            case 6:
                //FORWARD RIGHT
                drive = DRIVE_POWER;
                steer = -STEER_POWER;
                break;
            case 7:
                //BACKWARD LEFT
                drive = -DRIVE_POWER;
                steer = STEER_POWER;
                break;
            case 8:
                //BACKWARD RIGHT
                drive = -DRIVE_POWER;
                steer = -STEER_POWER;
                break;
            case 9:
                //ROTATION 0
                drive = 0;
                steer = 0;
                break;
            case 10:
                //ROTATION 180
                drive = 0;
                steer = Math.PI;
                break;
            case 11:
                //INCREMENT 45
                drive = 0;
                if (crimer.advanceIfElapsed(TURNGAP)){
                    steer += Math.PI/2;
                    steer = steer % (2 * Math.PI);
                }
                break;
            case 12:
                //INCREMENT 180
                drive = 0;
                if (crimer.advanceIfElapsed(TURNGAP)){
                    steer += Math.PI;
                    steer = steer % (2 * Math.PI);
                }
                break;
        }
        if (testCase > 8){
            super.setRawPowersWithAngle(drive, steer);
        }
        else{
            super.setRawPowers(drive, steer);
        }


        // System.out.println(testCase);
    }

    public void incrementTest(){
        testCase = (testCase == 12) ? 0 : testCase + 1;
        crime = 0;
    }

    public void decrementTest(){
        testCase = (testCase == 0) ? 12 : testCase - 1;
        crime = 0;
    }

    public int getTest(){
        return testCase;
    }

    public boolean getRunning(){
        return toRun;
    }

    public double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

}
