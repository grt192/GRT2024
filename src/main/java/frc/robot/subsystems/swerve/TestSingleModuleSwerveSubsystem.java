package frc.robot.subsystems.swerve;

import static frc.robot.Constants.TestSingleModuleSwerveConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/** Runs tests for individual swerve modules. */
public class TestSingleModuleSwerveSubsystem extends SingleModuleSwerveSubsystem {

    
    public static final double STEER_POWER = .4;
    public static final double DRIVE_POWER = 1;

    private int testCase;
    private boolean toRun;

    private double drive;
    private double steer;

    private Timer crimer;
    private double crime = 0;

    private Timer crimor;

    /** Makes a swerve subsystem for running tests on one module.
     *
     * @param module The module to run tests on.
     */
    public TestSingleModuleSwerveSubsystem(SwerveModule module) {
        super(module);

        testCase = 1;
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

        if (DriverStation.isDisabled()) {  
            System.out.println(module.getWrappedAngle().getRadians() * -1 + Math.PI / 2);
        }

        if (!toRun) {
            super.setRawPowers(0, 0);
            return;
        }

        // These are the different test cases as requested by Alex
        switch (testCase) {
            
            case 0:
                //WEAR IN GEARS
                if (crimer.advanceIfElapsed(crime)) {
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
                if (crimer.advanceIfElapsed(TestSingleModuleSwerveConstants.TURN_GAP)) {
                    steer += Math.PI / 2;
                    steer = steer % (2 * Math.PI);
                }
                break;
            case 12:
                //INCREMENT 180
                drive = 0;
                if (crimer.advanceIfElapsed(TestSingleModuleSwerveConstants.TURN_GAP)) {
                    steer += Math.PI;
                    steer = steer % (2 * Math.PI);
                }
                break;
            default:
                drive = 0;
                steer = 0;
        }

        if (testCase > 8) {
            setRawPowersWithAngle(drive, steer);
        } else {
            setRawPowers(drive, steer);
        }
    }

    /** Increase the test number by 1. */
    public void incrementTest() {
        testCase = (testCase == 12) ? 0 : testCase + 1;
        crime = 0;
    }

    /** Decrease the test number by 1. */
    public void decrementTest() {
        testCase = (testCase == 0) ? 12 : testCase - 1;
        crime = 0;
    }

    /** Get the selected test number.
     *
     * @return The test number
     */
    public int getTest() {
        return testCase;
    }

    /** Return if the module is running.
     *
     * @return Whether the module is running.
     */
    public boolean getRunning() {
        return toRun;
    }

    /** Toggle if the module is running. */
    public void toggleToRun() {
        toRun = !toRun;
        System.out.println(toRun);
    }

}
