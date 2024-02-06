package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class RotaryLEDLayer extends LEDLayer {

    public RotaryLEDLayer(int length) {
        super(length);
    }

    public void incrementColors(int inc) {
        for (int i = 0; i < colorArray.length - inc; i++) {
            setLED(i, getLEDColor(i + inc), getLEDOpacity(i + inc));
        }
        for (int i = colorArray.length - inc; i < colorArray.length; i++) {
            setLED(i, getLEDColor(i - colorArray.length + inc), getLEDOpacity(i - colorArray.length + inc));
        }
    }

    public Color getLEDColor(int i){
        return colorArray[i % ledLength];
    }

    public void setLED(int i, Color color, double opacity) {
        colorArray[(i + ledLength) % ledLength] = color;
        opacityArray[(i + ledLength) % ledLength] = opacity;
    }

    public void setLED(int i, Color color){
        setLED(i, color, 1);
    }

    /**
     * 
     * @param angle
     * @param radius
     * @param borderLength
     * @param color
     * @param opacity
     */
    public void setAngleGroup(double angle, int radius, int borderLength, Color color, double opacity) {

        //center
        int offset = (int) (ledLength * angle / (2 * Math.PI));

        fillColor(null);

        for (int i = 0; i < borderLength; i++) {
            setLED(offset - borderLength - radius - 1 + i, color, opacity * (i + 1) / (borderLength + 1));
        }

        for (int i = 0; i < radius * 2 + 1; i++) {
            // System.out.println(offset - radius - 1);
            setLED(offset - radius - 1 + i, color, opacity);
        }

        for (int i = 0; i < borderLength; i++) {
            setLED(offset + radius + i, color,  opacity * (1 - (i / (borderLength + 1.))));
        }
    }
}