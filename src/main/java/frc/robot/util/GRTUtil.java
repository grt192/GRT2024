package frc.robot.util;


import edu.wpi.first.wpilibj.util.Color;

/** Utility functions specific to our repo. */
public class GRTUtil {

    /** Rounds to two decimal points (x.yz). */
    public static double twoDecimals(double num) {
        return ((int) (num * 100)) / 100.d;
    }

    /** Scales a color by a scalar. */
    public static Color scaleColor(Color color, double scale) {
        return new Color((int) (color.red * scale * 255), 
                         (int) (color.green * scale * 255), 
                         (int) (color.blue * scale * 255));
    }

    /** Scales an OpacityColor by a scalar. */
    public static OpacityColor scaleColor(OpacityColor color, double scale) {
        return OpacityColor.blendColors(color, new OpacityColor(), scale);
    }
}
