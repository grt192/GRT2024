package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.TrackingTimer;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;
    // private final LEDStrip rishayStrip;
    private final LEDLayer baseLayer;
    private final RotaryLEDLayer driveAngleLayer;
    private final RotaryLEDLayer driveBackAngleLayer;
    private final RotaryLEDLayer rainbowLayer;
    private final LEDLayer aprilDetectedLayer;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private static final double BLINK_OFF_TO_ON_RATIO = 4;
    private boolean blinking = false;

    private final TrackingTimer aprilBlinkTimer = new TrackingTimer();
    private static final double APRIL_BLINK_DURATION_SECONDS = 0.05;

    private static final Timer fadeTimer = new Timer();
    private static final double COLOR_SENSOR_FADE_PERIOD_SECONDS = .5;
    private final double COLOR_PULSE_SPEED = .2;
    private boolean colorSensorOff = false;
    private double colorOffset = 0;

    
    private static final double INPUT_DEADZONE = 0.35;
    private static final int LEDS_PER_SEC = 150;

    private Color pieceColor = CUBE_COLOR;
    private Color manualColor = new Color(0, 0, 0);

    private boolean rainbow = false; // If the driver is directly controlling leds
    public boolean pieceGrabbed = false;
    private boolean enabled = false;

    
    private static final Color BASE_COLOR = scaleDownColorBrightness(new Color(255, 0, 0));
    private static final Color APRIL_COLOR = scaleDownColorBrightness(new Color(252, 255, 236));
    private static final Color CUBE_COLOR = scaleDownColorBrightness(new Color(192, 8, 254));
    private static final Color CONE_COLOR = scaleDownColorBrightness(new Color(255, 100, 0));
    private static final Color WHITE = scaleDownColorBrightness(new Color(255,255,255));
    private static final Color COLOR_SENSOR_OFF_COLOR = scaleDownColorBrightness(new Color(255, 0, 0));
    private static final Color BLUE = scaleDownColorBrightness(new Color(0,0,255));

    private final Timer ledTimer; // TODO: better naming

    private int offset = 0;
    private double driverHeading = 0.0;

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);

        baseLayer = new LEDLayer(LED_LENGTH);
        driveAngleLayer = new RotaryLEDLayer(LED_LENGTH);
        driveBackAngleLayer = new RotaryLEDLayer(LED_LENGTH);
        rainbowLayer = new RotaryLEDLayer(LED_LENGTH);
        aprilDetectedLayer = new LEDLayer(LED_LENGTH);

        blinkTimer = new Timer();
        ledTimer = new Timer();
        ledTimer.start();

        fadeTimer.start();

        // rishayStrip = new LEDStrip(1, LED_LENGTH);
    }

    @Override
    public void periodic() {

        // System.out.println("asdf");
        // Number of leds to increment each continuous led layer by
        int inc = Math.min((int) Math.ceil(ledTimer.get() * LEDS_PER_SEC), LED_LENGTH);
        ledTimer.reset();
        ledTimer.start();

        offset += inc;
        offset = offset % LED_LENGTH;

        // Update baseLayer - the piece color indicated by the mech driver, or the blink color if a piece
        // is held and we are blinking.
        baseLayer.fillColor(BASE_COLOR);
        // baseLayer.setLED(LED_LENGTH - 1, new Color(0,255,0));


        if(!DriverStation.isEnabled()){
            driverHeading = 2 * Math.PI * ((double) offset) / (double) LED_LENGTH;
            driveBackAngleLayer.fillColor(null);
        } else {
            driveBackAngleLayer.setAngleGroup(driverHeading + Math.PI, 5, 5, BLUE, .7);
        }
        driveAngleLayer.setAngleGroup(driverHeading, 5, 5, WHITE, .7);
        

        // Update aprilDetectedLayer - white pulses to indicate an april tag detection.
        if (!aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS) && aprilBlinkTimer.hasStarted()) {
            aprilDetectedLayer.fillGrouped(3, 6, 1, APRIL_COLOR, .7, offset);
        } else {
            aprilDetectedLayer.incrementColors(inc, null);
        }

        if(rainbow){
            rainbowLayer.setRainbow(offset);
        } else {
            rainbowLayer.fillColor(null);
        }

        // Add layers to buffer, set leds
        ledStrip.addLayer(baseLayer);
        ledStrip.addLayer(driveAngleLayer);
        ledStrip.addLayer(driveBackAngleLayer);
        ledStrip.addLayer(rainbowLayer);
        ledStrip.setBuffer();

        // rishayStrip.addLayer(baseLayer);
        // rishayStrip.addLayer(driveAngleLayer);
    }

    /**
     * Toggles whether drivers are manually controlling the color of the LEDs.
     */
    public void setRainbow(boolean rainbow) {
        this.rainbow = rainbow;
    }

    /**
     * Displays that an AprilTag has been detected by sending a pulse down the LEDs.
     */
    public void displayTagDetected() {
        aprilBlinkTimer.start();
        aprilBlinkTimer.advanceIfElapsed(APRIL_BLINK_DURATION_SECONDS * BLINK_OFF_TO_ON_RATIO);
    }

    /**
     * Cross-fades between two colors using a sinusoidal scaling function.
     * @param currentTimeSeconds The current elapsed time of fading.
     * @param periodSeconds The period of the fade function, in seconds.
     * @return The scale to set the opacity to
     */
    private static Color crossFadeWithTime(Color color, Color fadeColor, double periodSeconds) {
        // The [0.0, 1.0] brightness scale to scale the color by. Scale = 1/2 * cos(t) + 1/2 where
        // t is scaled to produce the desired period.
        double scale = 0.5 * Math.cos(fadeTimer.get() * 2 * Math.PI / periodSeconds) + 0.5;

        return new Color(
            color.red * scale + fadeColor.red * (1 - scale),
            color.green * scale + fadeColor.green * (1 - scale),
            color.blue * scale + fadeColor.blue * (1 - scale)
        );
    }

    /**
     * Scales a color's brightness by the BRIGHTNESS_SCALE_FACTOR.
     * @param color The color to scale down.
     * @return The scaled down color.
     */
    private static Color scaleDownColorBrightness(Color color) {
        return new Color(
            color.red * BRIGHTNESS_SCALE_FACTOR,
            color.green * BRIGHTNESS_SCALE_FACTOR,
            color.blue * BRIGHTNESS_SCALE_FACTOR
        );
    }

    /**
     * Sets the color of the LEDs commanded by driver input. If manual input is enabled, this sets the color
     * to the HSV color created by the angle of the joystick. Otherwise, set the color to CUBE if the joystick
     * is pushed right and CONE if it is pushed left.
     * 
     * @param x The x input of the joystick.
     * @param y The y input of the joystick.
     */
    public void setDriverColors(double x, double y){
        double angleRads = MathUtil.inputModulus(Math.atan2(y, x), 0, 2 * Math.PI);
        manualColor = Color.fromHSV(
            (int) (Math.toDegrees(angleRads) / 2.0),
            (int) 255,
            (int) (255 * BRIGHTNESS_SCALE_FACTOR)
        );

        if (x > INPUT_DEADZONE) {
            pieceColor = CUBE_COLOR;
        } else if (x < -INPUT_DEADZONE) {
            pieceColor = CONE_COLOR;
        }
    }

    public void setDriverHeading(double heading){
        driverHeading = heading;
    }
}