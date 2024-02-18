package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Util;

public class LEDLayer {
    protected final Color[] colorArray;
    protected final double[] opacityArray;
    protected final int ledLength;

    public LEDLayer(int length) {
        ledLength = length;
        colorArray = new Color[length];
        opacityArray = new double[length];
    }

    /**
     * Sets an LED at a specified index.
     * @param i The LED index to set.
     * @param color The color to set the LED at index i to (null is equivalent to transparent).
     * @param opacity The opacity to set the LED at index i to
     */
    public void setLED(int i, Color color, double opacity) {
        colorArray[i] = color;
        opacityArray[i] = opacity;
    }

    public void setLED(int i, Color color){
        setLED(i, color, 1);
    }

    /**
     * Gets the color of the LED at a specified index.
     * @param i The LED index to retrieve.
     * @return The color of the LED at index i.
     */
    public Color getLEDColor(int i) {
        return colorArray[i];
    }

    /**
     * Gets the opacity of the LED at a specified index
     * @param i The LED index to retrieve.
     * @return The opacity of the LED at index i.
     */
    public double getLEDOpacity(int i){
        return opacityArray[i];
    }

    /**
     * Moves the leds up by an increment
     * @param inc The number of leds to move up by
     * @param color The color to set at the bottom
     * @param opacity The opacity to set the new leds at
     */
    public void incrementColors(int inc, Color color, double opacity) {
        for (int i = 0; i < colorArray.length - inc; i++) {
            setLED(i, getLEDColor(i + inc), getLEDOpacity(i +inc));
        }
        for (int i = colorArray.length - inc; i < colorArray.length; i++) {
            setLED(i, color, opacity);
        }
    }

    public void incrementColors(int inc, Color color){
        incrementColors(inc, color, 1);
    }

    /**
     * Fills the layer with a solid color.
     * @param color The color to fill the layer with.
     * @param opacity The opacity to fill the layer with
     */
    public void fillColor(Color color, double opacity) {
        for (int i = 0; i < colorArray.length; i++) {
            setLED(i, color, opacity);
        }
    }

    public void fillColor(Color color){
        fillColor(color, 1);
    }

    /**
     * Fills the layer with alternating groups of "on" and "off" LEDs. "off" leds are set to null (transparent).
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param borderLength The length of a gradient border which fades in at the edge of each "on" length
     * @param color The color to set the "on" LEDs.
     * @param opacity The opacity of the "on" LEDs.
     * @param offset The number of LEDs to offset the base by
     */
    public void fillGrouped(int onGroupLength, int offGroupLength, int borderLength, Color color, double opacity, int offset) {
        for (int i = 0; i < colorArray.length; i++) {
            int ledNumInSegment = (i + offset) % (2 * borderLength + onGroupLength + offGroupLength);
            if (ledNumInSegment < borderLength){
                setLED(i, color, opacity *  (ledNumInSegment + 1) / (borderLength + 1));
            } else if (ledNumInSegment < onGroupLength + borderLength) {
                setLED(i, color, opacity);
            } else if(ledNumInSegment < onGroupLength + borderLength * 2){
                setLED(i, color, opacity * (1 - ((ledNumInSegment - onGroupLength - borderLength + 1.) / (borderLength + 1))));                
            } else {
                setLED(i, null, opacity);
            }
        }
    }

    public void fillGrouped(int onGroupLength, int offGroupLength, Color color){
        fillGrouped(onGroupLength, offGroupLength, 0, color, 1, 0);
    }

    /**
     * Resets the layer by setting all LEDs to null (transparent).
     */
    public void reset() {
        fillColor(null, 1);
    }

    public void scale(double factor){
        for(int i = 0; i < colorArray.length; i++){
            setLED(i, Util.scaleColor(getLEDColor(i), factor));
        }
    }
}