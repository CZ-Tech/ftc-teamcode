package org.firstinspires.ftc.teamcode.common.vision.processor;


import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

/**
 * An {@link ColorRange represents a 3-channel minimum/maximum
 * range for a given color space}
 */
public class ColorRange
{
    protected final ColorSpace colorSpace;
    protected final Scalar min;
    protected final Scalar max;

    // -----------------------------------------------------------------------------
    // DEFAULT OPTIONS
    // -----------------------------------------------------------------------------

    public static final ColorRange BLUE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16,   0, 155),
            new Scalar(255, 127, 255)
    );

    public static final ColorRange RED = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 176,  0),
            new Scalar(255, 255, 132)
    );

    public static final ColorRange YELLOW = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 128,   0),
            new Scalar(255, 170, 120)
    );

    public static final ColorRange GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32,   0,   0),
            new Scalar(255, 120, 133)
    );

    // -----------------------------------------------------------------------------
    // ROLL YOUR OWN
    // -----------------------------------------------------------------------------

    public ColorRange(ColorSpace colorSpace, Scalar min, Scalar max)
    {
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }
}
