package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public class Util {
    public static double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

    public static Color scaleColor(Color color, double scale){
        return new Color((int) (color.red * scale * 255), (int) (color.green * scale * 255), (int) (color.blue * scale * 255));
    }

    public static OpacityColor scaleColor(OpacityColor color, double scale){
        return OpacityColor.blendColors(color, new OpacityColor(), scale);
    }
}
