package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public class OpacityColor extends Color {
    public double opacity = 0;

    public OpacityColor(int red, int green, int blue, double opacity){
        super(red, green, blue);
        this.opacity = opacity;
    }

    public OpacityColor(double red, double green, double blue, double opacity){
        super(red, green, blue);
        this.opacity = opacity;
    }

    public OpacityColor(int red, int green, int blue){
        this(red, green, blue, 1.);
    }

    public OpacityColor(double red, double green, double blue){
        this(red, green, blue, 1.);
    }

    public OpacityColor(){
        this(0, 0, 0, 0.);

    }

    public OpacityColor(Color color, double opacity){
        this(color.red, color.green, color.blue, opacity);
    }

    public double getOpacity(){
        return opacity;
    }

    public Color getScaledColor(){
        return GRTUtil.scaleColor(this, opacity);
    }

    public OpacityColor withOpacity(double opacity){
        return new OpacityColor(red, green, blue, this.opacity * opacity);
    }

    public static OpacityColor fromHSV(int h, int s, int v, double opacity){
        return new OpacityColor(fromHSV(h, s, v), opacity);
    }

    public static OpacityColor blendColors(OpacityColor color1, OpacityColor color2, double secondColorFactor){
        return new OpacityColor(color2.red * secondColorFactor + color1.red * (1 - secondColorFactor),
                                color2.green * secondColorFactor + color1.green * (1 - secondColorFactor),
                                color2.blue * secondColorFactor + color1.blue * (1 - secondColorFactor),
                                color2.opacity * secondColorFactor + color2.opacity * (1 - secondColorFactor));
    }
}
