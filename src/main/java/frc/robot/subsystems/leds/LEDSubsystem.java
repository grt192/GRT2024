package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.NotePosition;
import frc.robot.util.OpacityColor;
import frc.robot.util.TrackingTimer;
import frc.robot.util.Util;

import static frc.robot.Constants.LEDConstants.*;


public class LEDSubsystem extends SubsystemBase {
    private final LEDStrip ledStrip;
    // private final LEDStrip rishayStrip;
    private final LEDLayer baseLayer;
    private final RotaryLEDLayer driveAngleLayer;
    private final RotaryLEDLayer driveBackAngleLayer;
    private final RotaryLEDLayer rainbowLayer;
    private final RotaryLEDLayer noteLayer;
    private final RotaryLEDLayer aprilDetectedLayer;

    private final Timer blinkTimer;
    private static final double BLINK_DURATION_SECONDS = 0.5;
    private static final double BLINK_OFF_TO_ON_RATIO = 4;
    private boolean blinking = false;

    private final TrackingTimer aprilBlinkTimer = new TrackingTimer();
    private static final double APRIL_BLINK_DURATION_SECONDS = 0.05;

    private static final Timer fadeTimer = new Timer();
    private static final int LEDS_PER_SEC = 100;

    private boolean rainbow = false; // If the driver is directly controlling leds
    public boolean pieceSeen = false;
    private boolean enabled = false;
    private NotePosition notePosition = NotePosition.NONE;

    private static final OpacityColor BASE_COLOR = new OpacityColor(255, 0, 0);
    private static final OpacityColor APRIL_COLOR = new OpacityColor(252, 255, 236);
    private static final OpacityColor NOTE_COLOR = new OpacityColor(255, 80, 0);
    private static final OpacityColor FRONT_COLOR = new OpacityColor(255, 255, 255);
    private static final OpacityColor BACK_COLOR = new OpacityColor(0, 0, 255);
    private static final OpacityColor COLOR_SENSOR_OFF_COLOR = new OpacityColor(255, 0, 0);
    private static final OpacityColor TRANSPARENT_COLOR = new OpacityColor();

    private final Timer ledTimer; // TODO: better naming

    private int offset = 0;
    private double driverHeading = 0.0;

    public LEDSubsystem() {
        ledStrip = new LEDStrip(LED_PWM_PORT, LED_LENGTH);

        baseLayer = new LEDLayer(LED_LENGTH);
        driveAngleLayer = new RotaryLEDLayer(LED_LENGTH);
        driveBackAngleLayer = new RotaryLEDLayer(LED_LENGTH);
        rainbowLayer = new RotaryLEDLayer(LED_LENGTH);
        noteLayer = new RotaryLEDLayer(LED_LENGTH);
        aprilDetectedLayer = new RotaryLEDLayer(LED_LENGTH);

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
        offset = offset % (LED_LENGTH * 120);

        // Update baseLayer - the piece color indicated by the mech driver, or the blink
        // color if a piece
        // is held and we are blinking.
        baseLayer.fillColor(BASE_COLOR);
        // baseLayer.setLED(LED_LENGTH - 1, new Color(0,255,0));

        if (!DriverStation.isEnabled()) {
            driverHeading = 2 * Math.PI * ((double) offset) / (double) LED_LENGTH;
            driveBackAngleLayer.fillColor(TRANSPARENT_COLOR);
            noteLayer.fillColor(TRANSPARENT_COLOR);
        } else {
            driveBackAngleLayer.setAngleGroup(driverHeading + Math.PI, 5, 5, BACK_COLOR.withOpacity(.7),
                    TRANSPARENT_COLOR);
        }
        driveAngleLayer.setAngleGroup(driverHeading, 5, 5, FRONT_COLOR.withOpacity(.7), TRANSPARENT_COLOR);

        if (rainbow) {
            rainbowLayer.setRainbow(offset);
        } else {
            rainbowLayer.fillColor(TRANSPARENT_COLOR);
        }

        OpacityColor noteColor = pieceSeen ? crossFadeWithTime(NOTE_COLOR, TRANSPARENT_COLOR, 1) : NOTE_COLOR;

        System.out.println(notePosition);

        switch (notePosition) {
            case NONE:
                noteLayer.fillColor(pieceSeen ? noteColor : new OpacityColor());
                break;
            case INTAKING:
                noteLayer.fillColor(TRANSPARENT_COLOR);
                noteLayer.setGroups(0., Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, true);
                noteLayer.setGroups(Math.PI, 2 * Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, false);
                break;
            case INTAKE_HOLDING:
                noteLayer.setAngleGroup(0, 30, 5, NOTE_COLOR, TRANSPARENT_COLOR);
                break;
            case TRANFER_TO_SHOOTER:
                noteLayer.fillColor(TRANSPARENT_COLOR);
                noteLayer.setGroups(0., Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, true);
                noteLayer.setGroups(Math.PI, 2 * Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, false);
                break;
            case TRANSFER_TO_INTAKE:
                noteLayer.fillColor(TRANSPARENT_COLOR);
                noteLayer.setGroups(0., Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, false);
                noteLayer.setGroups(Math.PI, 2 * Math.PI, 22, 12, 3, NOTE_COLOR, TRANSPARENT_COLOR, offset, true);
                break;
            case INTAKE_READY_TO_SHOOT:
                noteLayer.setAngleGroup(0, 30, 5, crossFadeWithTime(NOTE_COLOR, TRANSPARENT_COLOR, .5),
                        TRANSPARENT_COLOR);
                break;
            case AMPING:
                noteLayer.setAngleGroup(0, 30, 5, crossFadeWithTime(NOTE_COLOR, TRANSPARENT_COLOR, .5),
                        TRANSPARENT_COLOR);
                break;
            case SHOOTER_READY_TO_SHOOT:
                noteLayer.setAngleGroup(Math.PI, 30, 5, crossFadeWithTime(NOTE_COLOR, TRANSPARENT_COLOR, .5),
                        TRANSPARENT_COLOR);
                break;
            case SHOOTING:
                noteLayer.setAngleGroup(Math.PI, 30, 5, crossFadeWithTime(NOTE_COLOR, TRANSPARENT_COLOR, .2),
                        TRANSPARENT_COLOR);
                break;
            default:
                noteLayer.reset();
                break;
        }

        // Update aprilDetectedLayer - white pulses to indicate an april tag detection.
        if (!aprilBlinkTimer.hasElapsed(APRIL_BLINK_DURATION_SECONDS) && aprilBlinkTimer.hasStarted()) {
            aprilDetectedLayer.fillGrouped(3, 6, 1, APRIL_COLOR.withOpacity(.7), TRANSPARENT_COLOR, offset);
        } else {
            aprilDetectedLayer.incrementColors(inc, TRANSPARENT_COLOR);
        }

        // Add layers to buffer, set leds
        ledStrip.addLayer(baseLayer);
        ledStrip.addLayer(noteLayer);
        ledStrip.addLayer(driveAngleLayer);
        ledStrip.addLayer(driveBackAngleLayer);
        ledStrip.addLayer(rainbowLayer);
        ledStrip.setBuffer(BRIGHTNESS_SCALE_FACTOR);
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
     * 
     * @param currentTimeSeconds The current elapsed time of fading.
     * @param periodSeconds      The period of the fade function, in seconds.
     * @return The scale to set the opacity to
     */
    private static OpacityColor crossFadeWithTime(OpacityColor color, OpacityColor fadeColor, double periodSeconds) {
        // The [0.0, 1.0] brightness scale to scale the color by. Scale = 1/2 * cos(t) +
        // 1/2 where
        // t is scaled to produce the desired period.

        // mid workaround, TODO: find a better way

        double scale = 0.5 * Math.cos(fadeTimer.get() * 2 * Math.PI / periodSeconds) + 0.5;

        return new OpacityColor(
                color.red * scale + fadeColor.red * (1 - scale),
                color.green * scale + fadeColor.green * (1 - scale),
                color.blue * scale + fadeColor.blue * (1 - scale),
                color.opacity * scale + fadeColor.opacity * (1 - scale));
    }

    /**
     * Scales a color's brightness by the BRIGHTNESS_SCALE_FACTOR.
     * 
     * @param color The color to scale down.
     * @return The scaled down color.
     */
    private static Color scaleDownColorBrightness(Color color) {
        return Util.scaleColor(color, BRIGHTNESS_SCALE_FACTOR);
    }

    public void setDriverHeading(double heading) {
        driverHeading = heading;
    }

    public void setNoteMode(NotePosition notePosition) {
        this.notePosition = notePosition;
    }

    public void setNoteSeen(boolean noteSeen) {
        pieceSeen = noteSeen;
    }
}