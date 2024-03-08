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
    private final LEDLayer topLayer;

    private LightBarStatus status;

    private int rainbowOffset;
    private int inc;
    private int bounceOffset;
    private int bounceDirection;

    private double shooterSpeedPercentage;

    private final Timer ledTimer; // TODO: better naming

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

        topLayer = new LEDLayer(LEDConstants.LED_LENGTH);

        status = LightBarStatus.DORMANT;

        rainbowOffset = 0;
        inc = 1;
        bounceOffset = 0;
        bounceDirection = 1;

        shooterSpeedPercentage = 0.0;

        ledTimer = new Timer();
        ledTimer.start();
    }

    /** Periodic loop of subsystem.
     * 
     */
    public void periodic() {
        
        if (ledTimer.hasElapsed(0.05)) {
            
            rainbowOffset += inc;
            rainbowOffset = rainbowOffset % (LEDConstants.LED_LENGTH * 360);
            
            bounceOffset += (inc * bounceDirection);

            if ((bounceOffset >= LEDConstants.LED_LENGTH) || (bounceOffset <= 0)) {
                bounceOffset -= (inc * bounceDirection); // undo the increment that oversteps the array length
                bounceDirection *= -1; // switch the bounce direction
            }

        }

        switch (status) {
            case DORMANT: // LEDs --> RAINBOW BY DEFAULT
                topLayer.setRainbow(rainbowOffset);
                break;
            case AUTON: // LEDs --> BOUNCING BLUE DURING AUTON
                topLayer.setBounce(BLUE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case ENDGAME: // LEDs --> BOUNCING PURPLE DURING TELEOP
                topLayer.setBounce(PURPLE_ENDGAME_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case INTAKING: // LEDs --> BOUNCING ORANGE WHILE INTAKING
                topLayer.setBounce(ORANGE_NOTE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case HOLDING_NOTE: // LEDs --> BOUNCING GREEN WHEN HOLDING NOTE
                topLayer.setBounce(GREEN_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case AUTO_ALIGN: // LEDs --> BOUNCING BLUE WHILE AUTONOMOUS
                topLayer.setBounce(BLUE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case SHOOTER_SPIN_UP: // LEDs --> PROGRESS BAR (GREEN ON BLUE) WHILE SHOOTER SPINS UP
                topLayer.setProgressBar(BLUE_COLOR.withOpacity(0.5), GREEN_COLOR, shooterSpeedPercentage);
                break;
            default:
                topLayer.fillColor(RED_COLOR); 
                break;
        }

        ledStrip.addLayer(topLayer);
        ledStrip.setBuffer(1);

    }

    /** 
     * Set the state of the robot Light Bar.

     * @param status A LightBarStatus that represents the desired state.
     */
    public void setLightBarStatus(LightBarStatus status) {
        this.status = status;
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
