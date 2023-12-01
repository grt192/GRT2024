package frc.robot.subsystems.swerve;

import static frc.robot.Constants.TestSingleModuleSwerveConstants.*;

import edu.wpi.first.wpilibj.Timer;

public class TestSingleModuleSwerveSubsystem extends SingleModuleSwerveSubsystem {

    private int testCase;
    private boolean toRun;

    private double drive;
    private double steer;

    private Timer crimer;

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
        switch(testCase) {
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
        testCase = (testCase == 8) ? 0 : testCase + 1 ;
    }

    public void decrementTest(){
        testCase = (testCase == 0) ? 8 : testCase - 1;
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
