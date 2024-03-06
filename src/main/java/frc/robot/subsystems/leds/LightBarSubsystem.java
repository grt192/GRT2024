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

    private final Timer ledTimer; // TODO: better naming

    private static final OpacityColor TRANSPARENT_COLOR = new OpacityColor();
    private static final OpacityColor ORANGE_NOTE_COLOR = new OpacityColor(254, 80, 0); // used for intake status
    private static final OpacityColor PURPLE_ENDGAME_COLOR = new OpacityColor(192, 8, 254); // used for endgame 

    private static final OpacityColor RED_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor GREEN_COLOR = new OpacityColor(192, 8, 254); // used for shooter spin-up
    private static final OpacityColor BLUE_COLOR = new OpacityColor(0, 0, 255); // used for auto-align indicator
    

    public LightBarSubsystem() {

        ledStrip = new LEDStrip(LEDConstants.LED_PWM_PORT, LEDConstants.LED_LENGTH);

        baseLayer = new LEDLayer(LEDConstants.LED_LENGTH);
        topLayer = new LEDLayer(LEDConstants.LED_LENGTH);

        status = LightBarStatus.DORMANT;

        ledTimer = new Timer();
        ledTimer.start();
    }

    public void periodic() {
        
        baseLayer.fillColor(TRANSPARENT_COLOR);

        switch (status) {
            case DORMANT:
                break;
            case AUTON:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            case ENDGAME:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            case INTAKING:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            case HOLDING_NOTE:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            case AUTO_ALIGN:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            case SHOOTER_SPIN_UP:
                topLayer.fillColor(TRANSPARENT_COLOR);
                break;
            default:
                topLayer.reset();
                break;
        }

    }

    /** 
     * Set the state of the robot Light Bar.

     * @param status A LightBarStatus that represents the desired state.
     */
    public void setLightBarStatus(LightBarStatus status) {
        this.status = status;
    }
}
