package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.util.OpacityColor;

/** LightBarSubsystem represents the short strip of LEDs running across the shooter of the robot.
 *  Used for robot --> driver and driver --> human player signaling.
 * 
 */
public class LightBarSubsystem extends SubsystemBase {
    
    private final LEDStrip ledStrip;

    private final LEDLayer matchStatusLayer;
    private final LEDLayer mechLayer;
    private final LEDLayer autoAlignLayer;

    private LightBarStatus status;
    private LightBarStatus matchStatus;
    private LightBarStatus autoAlignStatus;
    private LightBarStatus mechStatus;

    private int rainbowOffset;
    private int inc;
    private int bounceOffset;
    private int bounceDirection;

    private double shooterSpeedPercentage;

    private final Timer ledTimer; // TODO: better naming
    private final Timer mechResetLEDTimer; 

    // private static final OpacityColor TRANSPARENT_COLOR = new OpacityColor();
    private static final OpacityColor ORANGE_NOTE_COLOR = new OpacityColor(254, 80, 0); // used for intake status
    private static final OpacityColor PURPLE_ENDGAME_COLOR = new OpacityColor(192, 8, 254); // used for endgame 

    private static final OpacityColor RED_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor GREEN_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor BLUE_COLOR = new OpacityColor(0, 0, 255); // used for auto-align indicator
    private static final OpacityColor WHITE_COLOR = new OpacityColor(255, 255, 255);
    
    /** Subsystem to manage a short strip of LEDs on the robot, used for robot->driver and driver->HP signaling.
     * 
     */
    public LightBarSubsystem() {

        ledStrip = new LEDStrip(LEDConstants.LED_PWM_PORT, LEDConstants.LED_LENGTH);

        matchStatusLayer = new LEDLayer(LEDConstants.LED_LENGTH);
        mechLayer = new LEDLayer(LEDConstants.LED_LENGTH);
        autoAlignLayer = new LEDLayer(LEDConstants.LED_LENGTH);

        status = LightBarStatus.DORMANT;
        matchStatus = LightBarStatus.DORMANT;
        autoAlignStatus = LightBarStatus.DORMANT;
        mechStatus = LightBarStatus.DORMANT;

        rainbowOffset = 0;
        inc = 1;
        bounceOffset = 0;
        bounceDirection = 1;

        shooterSpeedPercentage = 0.0;

        ledTimer = new Timer();
        ledTimer.start();
        
        // used to reset the shooter or intake LEDs 5 sec after we've picked up a note or reached shooter speed
        mechResetLEDTimer = new Timer(); 
    }

    /** Periodic loop of subsystem.
     * 
     */
    public void periodic() {
        
        if (ledTimer.advanceIfElapsed(0.05)) {
            
            rainbowOffset += inc;
            rainbowOffset = rainbowOffset % (LEDConstants.LED_LENGTH * 360);
            
            bounceOffset += (inc * bounceDirection);

            if ((bounceOffset >= LEDConstants.LED_LENGTH) || (bounceOffset <= 0)) {
                bounceOffset -= (inc * bounceDirection); // undo the increment that oversteps the array length
                bounceDirection *= -1; // switch the bounce direction
            }

        }

        switch (matchStatus) {
            case AUTON: // LEDs --> BOUNCING BLUE DURING AUTON
                matchStatusLayer.setBounce(BLUE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case ENDGAME: // LEDs --> BOUNCING PURPLE DURING TELEOP
                // matchStatusLayer.fillGrouped(4, 100, 16, WHITE_COLOR, PURPLE_ENDGAME_COLOR, bounceOffset + 10);
                matchStatusLayer.setBounce(PURPLE_ENDGAME_COLOR, WHITE_COLOR, bounceOffset);
                break;
            default:
                matchStatusLayer.setRainbow(rainbowOffset);
                break;
        }

        if (autoAlignStatus.equals(LightBarStatus.AUTO_ALIGN)) {
            autoAlignLayer.setBounce(BLUE_COLOR, WHITE_COLOR, bounceOffset);
        } else {
            autoAlignLayer.fillColor(new OpacityColor());
        }

        switch (mechStatus) {
            case INTAKING: // LEDs --> BOUNCING ORANGE WHILE INTAKING
                mechLayer.setBounce(ORANGE_NOTE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case HOLDING_NOTE: // LEDs --> BOUNCING GREEN WHEN HOLDING NOTE
                mechLayer.setBounce(GREEN_COLOR, WHITE_COLOR, bounceOffset);
                mechResetLEDTimer.start();
                break;
            case SHOOTER_SPIN_UP: // LEDs --> PROGRESS BAR (GREEN ON BLUE) WHILE SHOOTER SPINS UP
                mechLayer.setProgressBar(BLUE_COLOR, PURPLE_ENDGAME_COLOR, GREEN_COLOR, shooterSpeedPercentage);
                if (shooterSpeedPercentage >= 0.98) {
                    mechResetLEDTimer.start();
                }
                break;
            default:
                mechLayer.fillColor(new OpacityColor()); 
                break;
        }

        if (mechResetLEDTimer.hasElapsed(5.0)) {
            mechStatus = LightBarStatus.DORMANT;
            mechResetLEDTimer.stop();
            mechResetLEDTimer.reset();
        }

        /*
         * PRIORITIES
         * 
         * HIGH: 
         *  INTAKE/SHOOTER (MECH SPECIFIC)
         *  AUTO-ALIGN 
         *  MATCH STATUS (AUTON/ENDGAME/RAINBOW)
         * LOW: 
         * 
         */

        ledStrip.addLayer(matchStatusLayer);
        ledStrip.addLayer(autoAlignLayer);
        ledStrip.addLayer(mechLayer);

        ledStrip.setBuffer(1);

    }

    /** 
     * Set the state of the robot Light Bar.

     * @param status A LightBarStatus that represents the desired state.
     * @param statusType 0 = matchStatus, 1 = autoAlignStatus, 2 = mechStatus
     */
    public void setLightBarStatus(LightBarStatus status, int statusType) {
        
        if (statusType == 0) {
            this.matchStatus = status;
        } else if (statusType == 1) {
            this.autoAlignStatus = status;
        } else if (statusType == 2) {
            this.mechStatus = status;
        } else {
            System.out.println("[LightBarSubsystem] Invalid lightBar statusType provided.");
        }

        // this.status = status;
    }

    /** Gets the status of the light bar.
     *
     * @return The current light bar status.
     */
    public LightBarStatus getLightBarStatus() {
        return this.status;
    }

    /** Updates the coefficient storing the current percentage of shooter speed reached. 
     *  This is used to show the speed of the shooter as a 'progress bar' when we're spinning up the flywheel.
     *
     * @param newSpeed The new coefficient/percentage of shooter speed [0.0, 1.0].
     */
    public void updateShooterSpeedPercentage(double newSpeed) {
        this.shooterSpeedPercentage = newSpeed;
    }
}
