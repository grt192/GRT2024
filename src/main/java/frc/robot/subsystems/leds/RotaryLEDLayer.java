package frc.robot.subsystems.leds;

import static frc.robot.Constants.LEDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OpacityColor;

/** A circular LEDLayer. */
public class RotaryLEDLayer extends LEDLayer {

    /** A circular LEDLayer.
     *
     * @param length The length of the LEDLayer.
     */
    public RotaryLEDLayer(int length) {
        super(length);
    }

    /** Increment the colors by moving them forward.
     *
     * @param inc The number of leds to move forward by.
     */
    public void incrementColors(int inc) {
        for (int i = 0; i < colorArray.length - inc; i++) {
            setLED(i, getLEDColor(i + inc));
        }
        for (int i = colorArray.length - inc; i < colorArray.length; i++) {
            setLED(i, getLEDColor(i - colorArray.length + inc));
        }
    }

    /** Gets the OpacityColor of an LED.
     *
     * @param i The index of the LED.
     */
    public OpacityColor getLEDColor(int i) {
        return colorArray[i % ledLength];
    }

    /** Sets the LED at an index. Wraps around (i of -1 does the last led etc.)
     *
     * @param i The index of the LED to set.
     */
    public void setLED(int i, OpacityColor color) {
        colorArray[(i + ledLength) % ledLength] = color;
    }


    /** Sets a group of leds to a color.
     *
     * @param angle The Rotation2d of the group of leds.
     * @param radius The radius of the group in number of leds.
     * @param borderLength The border length of the group in number of leds.
     * @param color The OpacityColor of the group.
     * @param baseColor The OpacityColor to set the rest of the layer.
     */
    public void setAngleGroup(
        Rotation2d angle, int radius, int borderLength, 
        OpacityColor color, OpacityColor baseColor
    ) {

        //center
        int offset = (int) (ledLength * angle.getRadians() / (2 * Math.PI));

        fillColor(baseColor);

        for (int i = 0; i < borderLength; i++) {
            setLED(
                offset - borderLength - radius - 1 + i, 
                OpacityColor.blendColors(baseColor, color, (i + 1) / (borderLength + 1))
            );
        }

        for (int i = 0; i < radius * 2 + 1; i++) {
            setLED(offset - radius - 1 + i, color);
        }

        for (int i = 0; i < borderLength; i++) {
            setLED(offset + radius + i, OpacityColor.blendColors(baseColor, color, (1 - (i / (borderLength + 1.)))));
        }
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

    /** Sets groups of leds.
     *
     * @param startAngle [-2pi, 2pi] The minimum angle of the groups in radians.
     * @param endAngle [-2pi, 2pi] The maximum angle of the groups in radians
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param borderLength The length of the gradient between the "on" and "off" groups.
     * @param color The "on" group color.
     * @param baseColor The "off" group color.
     * @param offset The offset to shift the whole layer by.
     * @param inverted To invert the offset or not.
     */
    public void setGroups(
        double startAngle, double endAngle, int onGroupLength, int offGroupLength, int borderLength, 
        OpacityColor color, OpacityColor baseColor, int offset, boolean inverted
    ) {
        int period = (2 * borderLength + onGroupLength + offGroupLength);

        for (int i = (int) (colorArray.length * startAngle / (2 * Math.PI)); 
            i < colorArray.length * endAngle / (2 * Math.PI); i++) {

            int ledNumInSegment = (((i + (inverted ? -offset : offset)) % period) + period) % period;

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

    /** Set groups of leds.
     *
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param borderLength The length of the gradient between the "on" and "off" groups.
     * @param color The "on" group color.
     * @param baseColor The "off" group color.
     * @param offset The offset to shift the whole layer by.
     */
    public void setGroups(
        int onGroupLength, int offGroupLength, int borderLength, 
        OpacityColor color, OpacityColor baseColor, int offset
    ) {
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, borderLength, color, baseColor, offset, false);
    }

    /** Set groups of leds.
     *
     * @param onGroupLength The length of the "on" group.
     * @param offGroupLength The length of the "off" group.
     * @param color The "on" group color.
     * @param baseColor The "off" group color.
     * @param offset The offset to shift the whole layer by.
     */
    public void setGroups(
        int onGroupLength, int offGroupLength, 
        OpacityColor color, OpacityColor baseColor, int offset
    ) {
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, 0, color, baseColor, offset, false);
    }
}