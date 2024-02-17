package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.Constants.LEDConstants.*;

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

    public void setRainbow(int offset){
        for(int i = 0; i < colorArray.length; i++){
            setLED(i + offset, Color.fromHSV(
                (int) (((double) i) * 180.0 / colorArray.length),
                (int) 255,
                (int) (255 * BRIGHTNESS_SCALE_FACTOR)
            ));
        }
    }

    public void setGroups(double startAngle, double endAngle, int onGroupLength, int offGroupLength, int borderLength, Color color, double opacity, int offset, boolean inverted){
        for (int i = (int) (colorArray.length * startAngle / (2 * Math.PI)); i < colorArray.length * endAngle / (2 * Math.PI); i++) {
            int ledNumInSegment = (i + (inverted ? offset : -offset)) % (2 * borderLength + onGroupLength + offGroupLength);
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

    public void setGroups(int onGroupLength, int offGroupLength, int borderLength, Color color, double opacity, int offset){
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, borderLength, color, opacity, offset, false);
    }

    public void setGroups(int onGroupLength, int offGroupLength, Color color, double opacity, int offset){
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, 0, color, opacity, offset, false);
    }
}