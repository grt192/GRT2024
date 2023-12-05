package frc.robot.subsystems.swerve;

import static frc.robot.Constants.TestSingleModuleSwerveConstants.*;

import edu.wpi.first.wpilibj.Timer;

public class TestSingleModuleSwerveSubsystem extends SingleModuleSwerveSubsystem {

    private int testCase;
    private boolean toRun;

    private double drive;
    private double steer;

    private Timer crimer;
    private double crime = 0;

    public TestSingleModuleSwerveSubsystem(SwerveModule module){
        super(module);

        testCase = 1;
        toRun = false;

        steer = 0;
        drive = 0;

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
                    drive = POWER * (Math.floor(Math.random() * 2) * 2) - 1; //Either 1 or -1
                    steer = POWER * (Math.floor(Math.random() * 2) * 2) - 1;
                    
                    crime = Math.random() * 20 + 20;
                }
            case 1:
                //DRIVE FORWARD
                drive = POWER;
                steer = 0;
            case 2:
                //DRIVE BACKWARD
                drive = -POWER;
                steer = 0;
            case 3:
                //STEER LEFT
                drive = 0;
                steer = POWER;
            case 4:
                //STEER RIGHT
                drive = 0;
                steer = -POWER;
            case 5:
                //FORWARD LEFT
                drive = steer = POWER;
            case 6:
                //FORWARD RIGHT
                drive = POWER;
                steer = -POWER;
            case 7:
                //BACKWARD LEFT
                drive = -POWER;
                steer = POWER;
            case 8:
                //BACKWARD RIGHT
                drive = steer = -POWER;
            case 9:
                //ROTATION 0
                drive = steer = 0;
            case 10:
                //ROTATION 180
                drive = 0;
                steer = Math.PI;
            case 11:
                //INCREMENT 45
                drive = 0;
                if (crimer.advanceIfElapsed(TURNGAP)){
                    steer += Math.PI/2;
                    steer = steer % (2 * Math.PI);
                }
            case 12:
                //INCREMENT 180
                drive = 0;
                if (crimer.advanceIfElapsed(TURNGAP)){
                    steer += Math.PI;
                    steer = steer % (2 * Math.PI);
                }
        }
        if (testCase > 8){
            super.setRawPowersWithAngle(drive, steer);
        }
        else{
            super.setRawPowers(drive, steer);
        }
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
