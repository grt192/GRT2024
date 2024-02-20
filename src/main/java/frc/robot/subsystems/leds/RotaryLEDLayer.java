package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.OpacityColor;

import static frc.robot.Constants.LEDConstants.*;

public class RotaryLEDLayer extends LEDLayer {

    public RotaryLEDLayer(int length) {
        super(length);
    }

    public void incrementColors(int inc) {
        for (int i = 0; i < colorArray.length - inc; i++) {
            setLED(i, getLEDColor(i + inc));
        }
        for (int i = colorArray.length - inc; i < colorArray.length; i++) {
            setLED(i, getLEDColor(i - colorArray.length + inc));
        }
    }

    public OpacityColor getLEDColor(int i){
        return colorArray[i % ledLength];
    }

    public void setLED(int i, OpacityColor color) {
        colorArray[(i + ledLength) % ledLength] = color;
    }


    /**
     * 
     * @param angle
     * @param radius
     * @param borderLength
     * @param color
     * @param opacity
     */
    public void setAngleGroup(double angle, int radius, int borderLength, OpacityColor color, OpacityColor baseColor) {

        //center
        int offset = (int) (ledLength * angle / (2 * Math.PI));

        fillColor(null);

        for (int i = 0; i < borderLength; i++) {
            setLED(offset - borderLength - radius - 1 + i, OpacityColor.blendColors(baseColor, color, (i + 1) / (borderLength + 1)));
        }

        for (int i = 0; i < radius * 2 + 1; i++) {
            // System.out.println(offset - radius - 1);
            setLED(offset - radius - 1 + i, color);
        }

        for (int i = 0; i < borderLength; i++) {
            setLED(offset + radius + i, OpacityColor.blendColors(baseColor, color, (1 - (i / (borderLength + 1.)))));
        }
    }

    public void setRainbow(int offset){
        for(int i = 0; i < colorArray.length; i++){
            setLED(i + offset, OpacityColor.fromHSV(
                (int) (((double) i) * 180.0 / colorArray.length),
                (int) 255,
                (int) (255 * BRIGHTNESS_SCALE_FACTOR),
                1.
            ));
        }
    }

    public void setGroups(double startAngle, double endAngle, int onGroupLength, int offGroupLength, int borderLength, OpacityColor color, OpacityColor baseColor, int offset, boolean inverted){
        int period = (2 * borderLength + onGroupLength + offGroupLength);
        for (int i = (int) (colorArray.length * startAngle / (2 * Math.PI)); i < colorArray.length * endAngle / (2 * Math.PI); i++) {
            int ledNumInSegment = (((i + (inverted ? -offset : offset)) % period) + period) % period;
            if (ledNumInSegment < borderLength){
                setLED(i, OpacityColor.blendColors(baseColor, color, (ledNumInSegment + 1) / (borderLength + 1)));
            } else if (ledNumInSegment < onGroupLength + borderLength) {
                setLED(i, color);
            } else if(ledNumInSegment < onGroupLength + borderLength * 2){
                setLED(i, OpacityColor.blendColors(baseColor, color, (1 - ((ledNumInSegment - onGroupLength - borderLength + 1.) / (borderLength + 1)))));                
            } else {
                setLED(i, baseColor);
            }
        }
    }

    public void setGroups(int onGroupLength, int offGroupLength, int borderLength, OpacityColor color, OpacityColor baseColor, int offset){
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, borderLength, color, baseColor, offset, false);
    }

    public void setGroups(int onGroupLength, int offGroupLength, OpacityColor color, OpacityColor baseColor, int offset){
        setGroups(0, Math.PI * 2, onGroupLength, offGroupLength, 0, color, baseColor, offset, false);
    }
}