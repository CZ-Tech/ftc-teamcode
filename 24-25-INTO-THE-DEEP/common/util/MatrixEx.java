package org.firstinspires.ftc.teamcode.common.util;

public class MatrixEx {
    public double[][] data;

    // 构造函数
    public MatrixEx(double[][] data) {
        this.data = data;
    }

    public void set(double[][] data) {
        this.data = data;
    }

    public void set(MatrixEx other) {
        this.data = other.data;
    }


    // 矩阵加法
    public MatrixEx add(MatrixEx other) {
        int rows = data.length;
        int cols = data[0].length;
        double[][] result = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return new MatrixEx(result);
    }

    // 矩阵乘法
    public MatrixEx multiply(MatrixEx other) {
        int rows = data.length;
        int cols = other.data[0].length;
        int innerDim = data[0].length;
        double[][] result = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                for (int k = 0; k < innerDim; k++) {
                    result[i][j] += this.data[i][k] * other.data[k][j];
                }
            }
        }
        return new MatrixEx(result);
    }

}
