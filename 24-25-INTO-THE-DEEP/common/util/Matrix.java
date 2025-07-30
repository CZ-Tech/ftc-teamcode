package org.firstinspires.ftc.teamcode.common.util;

public class Matrix {
//    private final Robot robot = Robot.INSTANCE;

    public double[] times(double[][] matrix1, double[] matrix2) {
        double[] result = new double[2];
        int i = 0;
        for (double[] x: matrix1) {
            result[i] = x[0] * matrix2[0] + x[1] * matrix2[1];
            i++;
        }
        return result;
    }
}
