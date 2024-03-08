package frc.robot.subsystems.leds;

import frc.robot.util.OpacityColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.util.GRTUtil;

/** One layer of the LED strip subsystem. */
public class LEDLayer {
    protected final OpacityColor[] colorArray;
    protected final int ledLength;

    /** One ledStrip layer.
     *
     * @param length The number of leds in the layer.
     */
    public LEDLayer(int length) {
        ledLength = length;
        colorArray = new OpacityColor[length];
    }

    /** Sets an LED at a specified index.
     *
     * @param i The LED index to set.
     * @param color The color containing am RGBA value
     */
    public void setLED(int i, OpacityColor color) {
        colorArray[i] = color;
    }

    /** Gets the color of the LED at a specified index.
     *
     * @param i The LED index to retrieve.
     * @return The color of the LED at index i.
     */
    public OpacityColor getLEDColor(int i) {
        return colorArray[i];
    }

    /** Sets this layer to a rainbow.
     *
     * @param offset The offset of the rainbow. Used to make it spin.
     */
    public void setRainbow(int offset) {
        for (int i = 0; i < colorArray.length; i++) {
            setLED(i + offset, OpacityColor.fromHSV(
                (int) (((double) i) * 180.0 / colorArray.length),
                (int) 255,
                (int) (255 * LEDConstants.BRIGHTNESS_SCALE_FACTOR),
                1.
            ));
        }
    }

    /** Sets this layer to a bounce effect.

     * @param baseColor The primary color of the bounce effect.
     * @param peakColor The color to use for the 'peak' of the bounce.
     * @param offset The offset of the bounce effect.
     */
    public void setBounce(OpacityColor baseColor, OpacityColor peakColor, int offset) {
        for (int i = 0; i < colorArray.length; i++) {
            
            double distance = Math.abs(i - offset);

            setLED(i, OpacityColor.blendColors(baseColor, peakColor, (distance / colorArray.length)));
        }

    }

    /** Moves the leds up by an increment.
     *
     * @param inc The number of leds to move up by
     * @param color The color & opacity to set at the bottom
     */
    public void incrementColors(int inc, OpacityColor color) {
        for (int i = 0; i < colorArray.length - inc; i++) {
            setLED(i, getLEDColor(i + inc));
        }
        for (int i = colorArray.length - inc; i < colorArray.length; i++) {
            setLED(i, color);
        }
    }

    /** Fills the layer with a solid color.
     *
     * @param color The color & opacity to fill the layer with.
     */
    public void fillColor(OpacityColor color) {
        for (int i = 0; i < colorArray.length; i++) {
            setLED(i, color);
        }
    }


    /** Fills the layer with alternating groups of "on" and "off" LEDs. "off" leds are set to the base color.
     *
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param borderLength The length of a gradient border which fades in at the edge of each "on" length
     * @param color The color & opacity to set the "on" LEDs.
     * @param baseColor The color & opacity to set "off" leds
     * @param offset The number of LEDs to offset the base by
     */
    public void fillGrouped(int onGroupLength, int offGroupLength, int borderLength, 
        OpacityColor color, OpacityColor baseColor, int offset) {
        
        for (int i = 0; i < colorArray.length; i++) {
            int ledNumInSegment = (i + offset) % (2 * borderLength + onGroupLength + offGroupLength);
            if (ledNumInSegment < borderLength) {
                setLED(i, OpacityColor.blendColors(baseColor, color, (ledNumInSegment + 1) / (borderLength + 1)));
            } else if (ledNumInSegment < onGroupLength + borderLength) {
                setLED(i, color);
            } else if (ledNumInSegment < onGroupLength + borderLength * 2) {
                setLED(i, OpacityColor.blendColors(
                    baseColor, 
                    color, 
                    (1 - ((ledNumInSegment - onGroupLength - borderLength + 1.) / (borderLength + 1)))
                ));                
            } else {
                setLED(i, baseColor);
            }
        }
    }

    /** Fills the layer with alternating groups of "on" and "off" LEDs. "off" leds are set to the base color.
     *
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param color The color & opacity to set the "on" LEDs.
     */
    public void fillGrouped(int onGroupLength, int offGroupLength, OpacityColor color) {
        fillGrouped(onGroupLength, offGroupLength, 0, color, new OpacityColor(), 0);
    }

    /** Resets the layer by setting all LEDs to null (transparent). */
    public void reset() {
        fillColor(new OpacityColor());
    }

    /** Scales the entire layer by a factor.
     *
     * @param factor The factor to scale the layer by.
     */
    public void scale(double factor) {
        for (int i = 0; i < colorArray.length; i++) {
            setLED(i, GRTUtil.scaleColor(getLEDColor(i), factor));
        }
    }
}