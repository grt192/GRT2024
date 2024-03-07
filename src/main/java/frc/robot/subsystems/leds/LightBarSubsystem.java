package frc.robot.subsystems.leds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.util.OpacityColor;

public class LightBarSubsystem extends SubsystemBase {
    
    private final LEDStrip ledStrip;
    private final LEDLayer baseLayer;
    private final LEDLayer topLayer;

    private LightBarStatus status;

    private int rainbowOffset;
    private int inc;
    private int bounceOffset;
    private int bounceDirection;

    private final Timer ledTimer; // TODO: better naming

    private static final OpacityColor TRANSPARENT_COLOR = new OpacityColor();
    private static final OpacityColor ORANGE_NOTE_COLOR = new OpacityColor(254, 80, 0); // used for intake status
    private static final OpacityColor PURPLE_ENDGAME_COLOR = new OpacityColor(192, 8, 254); // used for endgame 

    private static final OpacityColor RED_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor GREEN_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor BLUE_COLOR = new OpacityColor(0, 0, 255); // used for auto-align indicator
    private static final OpacityColor WHITE_COLOR = new OpacityColor(255, 255, 255);
    

    public LightBarSubsystem() {

        ledStrip = new LEDStrip(LEDConstants.LED_PWM_PORT, LEDConstants.LED_LENGTH);

        baseLayer = new LEDLayer(LEDConstants.LED_LENGTH);
        topLayer = new LEDLayer(LEDConstants.LED_LENGTH);

        status = LightBarStatus.DORMANT;

        rainbowOffset = 0;
        inc = 1;
        bounceOffset = 0;
        bounceDirection = 1;

        ledTimer = new Timer();
        ledTimer.start();
    }

    public void periodic() {
        
        baseLayer.fillColor(TRANSPARENT_COLOR);
        
        rainbowOffset += inc;
        rainbowOffset = rainbowOffset % (LEDConstants.LED_LENGTH * 360);
        
        bounceOffset += (inc * bounceDirection);
        if ((bounceOffset >= LEDConstants.LED_LENGTH) || (bounceOffset <= 0)) {
            bounceOffset -= (inc * bounceDirection); // undo the increment that oversteps the array length
            bounceDirection *= -1; // switch the bounce direction
        }

        switch (status) {
            case DORMANT:
                break;
            case AUTON:
                topLayer.setRainbow(rainbowOffset);
                break;
            case ENDGAME:
                topLayer.setBounce(PURPLE_ENDGAME_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case INTAKING:
                topLayer.setBounce(ORANGE_NOTE_COLOR, RED_COLOR, bounceOffset);
                break;
            case HOLDING_NOTE:
                topLayer.fillColor(ORANGE_NOTE_COLOR);
                break;
            case AUTO_ALIGN:
                topLayer.setBounce(BLUE_COLOR, WHITE_COLOR, bounceOffset);
                break;
            case SHOOTER_SPIN_UP:
                topLayer.setBounce(GREEN_COLOR, WHITE_COLOR, bounceOffset);
                break;
            default:
                topLayer.reset();
                break;
        }

        ledStrip.addLayer(baseLayer);
        ledStrip.addLayer(topLayer);

    }

    /** 
     * Set the state of the robot Light Bar.

     * @param status A LightBarStatus that represents the desired state.
     */
    public void setLightBarStatus(LightBarStatus status) {
        this.status = status;
    }
}
