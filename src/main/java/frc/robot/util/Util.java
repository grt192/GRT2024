package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public class Util {
    public static double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

    public static Color scaleColor(Color color, double scale){
        return new Color((int) (color.red * scale), (int) (color.green * scale), (int) (color.blue * scale));
    }
}
