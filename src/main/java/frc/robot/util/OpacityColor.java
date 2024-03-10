package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

/** An extension to color that includes opacity. */
public class OpacityColor extends Color {
    public double opacity = 0;

    /**
     * Constructs an RGBA OpacityColor.
     *
     * @param red Red value int [0,255]
     * @param green Green value int [0, 255]
     * @param blue Blue value int [0, 255]
     * @param opacity Opacity value double [0, 1]
     */
    public OpacityColor(int red, int green, int blue, double opacity) {
        super(red, green, blue);
        this.opacity = opacity;
    }

    /**
     * Constructs an RGBA OpacityColor.
     *
     * @param red Red value double [0,1]
     * @param green Green value double [0, 1]
     * @param blue Blue value double [0, 1]
     * @param opacity Opacity value double [0, 1]
     */
    public OpacityColor(double red, double green, double blue, double opacity) {
        super(red, green, blue);
        this.opacity = opacity;
    }

    /**
     * Constructs an RGBA OpacityColor with full opacity.
     *
     * @param red Red value int [0,255]
     * @param green Green value int [0, 255]
     * @param blue Blue value int [0, 255]
     */
    public OpacityColor(int red, int green, int blue) {
        this(red, green, blue, 1.);
    }

    /**
     * Constructs an RGBA OpacityColor with full opacity.
     *
     * @param red Red value double [0,1]
     * @param green Green value double [0, 1]
     * @param blue Blue value double [0, 1]
     */
    public OpacityColor(double red, double green, double blue) {
        this(red, green, blue, 1.);
    }

    /** Constructs a transparent and black OpacityColor. */
    public OpacityColor() {
        this(0, 0, 0, 0.);

    }

    /** Constructs an OpacityColor from a regular color and an opacity.
     *
     * @param color The regular color
     * @param opacity Opacity values [0, 1]
     */
    public OpacityColor(Color color, double opacity) {
        this(color.red, color.green, color.blue, opacity);
    }

    /**
     * Get the opacity.
     *
     * @return The opacity of the OpacityColor [0, 1]
     */
    public double getOpacity() {
        return opacity;
    }

    /**
     * Get the equivalent scaled color.
     *
     * @return A regular color scaled to the opacity.
     */
    public Color getScaledColor() {
        return GRTUtil.scaleColor(this, opacity);
    }

    /**
     * Get the OpacityColor if applied with another opacity on top.
     *
     * @param opacity The opacity to add.
     * @return The opacity color with both opacities multiplied.
     */
    public OpacityColor withOpacity(double opacity) {
        return new OpacityColor(red, green, blue, this.opacity * opacity);
    }

    /**
     * Constructs an OpacityColor from HSV.
     *
     * @param h The hue value [0, 180)
     * @param s The saturation value [0, 255]
     * @param v The value value [0, 255]
     * @param opacity The opacity values [0, 1]
     * @return The OpacityColor created
     */
    public static OpacityColor fromHSV(int h, int s, int v, double opacity) {
        return new OpacityColor(fromHSV(h, s, v), opacity);
    }

    /**
     * Blends two colors with a scale.
     *
     * @param color1 The first color to blend.
     * @param color2 The second color to blend.
     * @param secondColorFactor The factor to add in the second color. 1 will be the second color. 0 is the first color.
     * @return The OpacityColor after blending.
     */
    public static OpacityColor blendColors(OpacityColor color1, OpacityColor color2, double secondColorFactor) {
        return new OpacityColor(color2.red * secondColorFactor + color1.red * (1 - secondColorFactor),
                                color2.green * secondColorFactor + color1.green * (1 - secondColorFactor),
                                color2.blue * secondColorFactor + color1.blue * (1 - secondColorFactor),
                                color2.opacity * secondColorFactor + color2.opacity * (1 - secondColorFactor));
    }
}
