package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.OpacityColor;
import frc.robot.util.Util;

/** An LEDStrip which you can add ledLayers to. */
public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int ledLength;

    /** An LEDStrip which you can add ledLayers to.
     *
     * @param ledPort The PWM port that LEDs are plugged into on the roborio.
     * @param ledLength The number of LEDs in the chain.
     */
    public LEDStrip(int ledPort, int ledLength) {
        this.ledLength = ledLength;
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    /** Sets the LED data to the current buffer. */
    public void setBuffer() {
        led.setData(ledBuffer);
    }

    /** Sets the LED data to the current buffer with a scaling factor.
     *
     * @param brightnessFactor [0.0, 1.0] The factor to scale the brightness by.
     */
    public void setBuffer(double brightnessFactor) {
        for (int i = 0; i < ledLength; i++) {
            ledBuffer.setLED(i, Util.scaleColor(ledBuffer.getLED(i), brightnessFactor));
        }
        led.setData(ledBuffer);
    }

    /** Applies an `LEDLayer` on top of the LED buffer.
     *
     * @param layer The layer to be added on top of the buffer. Null leds are considered transparent and "fall through"
     *     to the previous layer's color.
     */
    public void addLayer(LEDLayer layer) {
        for (int i = 0; i < ledLength; i++) {
            if (layer.getLEDColor(i) != null) {
                ledBuffer.setLED(i, calcColorWithOpacity(ledBuffer.getLED(i), layer.getLEDColor(i)));
            }
        }
    }

    /** Calculates the desired color when led layering.
     *
     * @param baseColor The color of the current led layer.
     * @param topColor The OpacityColor of the led being added on top.
     * @return The color of the led layers combined.
     */
    private Color calcColorWithOpacity(Color baseColor, OpacityColor topColor) {
        double r = (((1 - topColor.opacity) * baseColor.red) + (topColor.opacity * topColor.red));
        double g = (((1 - topColor.opacity) * baseColor.green) + (topColor.opacity * topColor.green));
        double b = (((1 - topColor.opacity) * baseColor.blue) + (topColor.opacity * topColor.blue));

        return (new Color(r, g, b));
    }
}