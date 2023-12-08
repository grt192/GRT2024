package frc.robot.subsystems.swerve;

import static frc.robot.Constants.TestSingleModuleSwerveConstants.*;

import edu.wpi.first.wpilibj.Timer;

public class TestSingleModuleSwerveSubsystem extends SingleModuleSwerveSubsystem {

    
    public static final double STEER_POWER = .1;
    public static final double DRIVE_POWER = .2;

    private int testCase;
    private boolean toRun;

    private double drive;
    private double steer;

    private Timer crimer;
    private double crime = 0;

    public TestSingleModuleSwerveSubsystem(SwerveModule module){
        super(module);

        testCase = 0;
        toRun = false;

        steer = 0;
        drive = 0;

        crimer = new Timer();
        crimer.start();
    }

    @Override
    public void periodic() {
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

        System.out.print(module.getWrappedAngle());
        System.out.println(testCase);

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

    public void toggletoRun(){
        toRun = !toRun;
    }

    public int getTest(){
        return testCase;
    }

    public boolean getRunning(){
        return toRun;
    }

}
