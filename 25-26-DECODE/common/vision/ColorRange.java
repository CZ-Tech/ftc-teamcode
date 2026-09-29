package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

public class ColorRange {
    public ColorSpace colorSpace;
    public Scalar min;
    public Scalar max;

    // -----------------------------------------------------------------------------
    // DEFAULT OPTIONS
    // -----------------------------------------------------------------------------

    public static ColorRange BLUE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 16,   0, 155),
            new Scalar(255, 127, 255)
    );

    public static ColorRange RED = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 176,  0),
            new Scalar(255, 255, 132)
    );

    public static ColorRange YELLOW = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 128,   0),
            new Scalar(255, 170, 120)
    );

    public static ColorRange GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32,   0,   0),
            new Scalar(255, 120, 133)
    );

    public static ColorRange ARTIFACT_GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32,  50, 118),
            new Scalar(255, 105, 145)
    );

    public static ColorRange ARTIFACT_PURPLE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 32, 135, 135),
            new Scalar(255, 155, 169)
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
