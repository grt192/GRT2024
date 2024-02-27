package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class GRTUtil {
    public static double twoDecimals(double num){
        return ((int) (num * 100)) / 100.d;
    }

    public static Color scaleColor(Color color, double scale){
        return new Color((int) (color.red * scale * 255), (int) (color.green * scale * 255), (int) (color.blue * scale * 255));
    }

    public static OpacityColor scaleColor(OpacityColor color, double scale){
        return OpacityColor.blendColors(color, new OpacityColor(), scale);
    }

    public static Command getBinaryCommandChoice(BooleanSupplier supplier, Command commandA, Command commandB) {
        return supplier.getAsBoolean() ? commandB : commandA;
    }
}
