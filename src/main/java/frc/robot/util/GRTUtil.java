package frc.robot.util;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    /**
     * Mirrors a pose2d across the center line.
     *
     * @param pose The pose to mirror.
     * @param mirror The function to check whether to mirror or not. 
     * @return The mirror (or not) pose.
     */
    public static Pose2d mirrorAcrossField(Pose2d pose, BooleanSupplier mirror) {
        return new Pose2d(
            mirror.getAsBoolean() ? 16.52 - pose.getX() : pose.getX(),
            pose.getY(),
            mirror.getAsBoolean() ? new Rotation2d(Math.PI).minus(pose.getRotation()) : pose.getRotation()
        );
    }
}
