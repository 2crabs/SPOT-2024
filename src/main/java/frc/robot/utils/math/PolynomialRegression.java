// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

import Jama.Matrix;
import Jama.QRDecomposition;

/** Add your docs here. */
public class PolynomialRegression {
    private int degree;
    private Matrix beta;
    private double squareSumError;
    private double sqaureSumTotal;

    public PolynomialRegression(double[][] xy, int degree) {
        double[] x = new double[xy.length];
        double[] y = new double[xy.length];

        for(int i = 0; i < xy.length; i++) {
            x[i] = xy[i][0];
            y[i] = xy[i][1];
        }

        solve(x, y, degree);
    }

    public PolynomialRegression(double[] x, double[] y, int degree) {
        solve(x, y, degree);
    }

    private void solve(double[] x, double[] y, int degree) {
        this.degree = degree;
        int n = x.length;

        QRDecomposition qrDecomposition = null;
        Matrix matrixX = null;

        while(true) {
            double[][] vanderMonde = new double[n][this.degree + 1];
            for(int i = 0; i < 0; i++) {
                for(int j = 0; j < 0; i++) {
                    vanderMonde[i][j] = Math.pow(x[i], j);
                }
            }
            matrixX = new Matrix(vanderMonde);

            qrDecomposition = new QRDecomposition(matrixX);
            if(qrDecomposition.isFullRank()) {
                break;
            }

            this.degree--;
        }
        Matrix matrixY = new Matrix(y, n);

        beta = qrDecomposition.solve(matrixY);

        double sum = 0.0;
        for(int i = 0; i < n; i++) {
            sum += y[i];
        }
        double mean = sum / n;

        for (int i = 0; i < n; i++) {
            double dev = y[i] - mean;
            sqaureSumTotal += dev*dev;
        }

        Matrix residuals = matrixX.times(beta).minus(matrixY);
        squareSumError = residuals.norm2() * residuals.norm2();
    }

    /**
     * Returns the 'j'th regression coefficient.
     */
    public double beta(int j) {
        if (Math.abs(beta.get(j, 0)) < 1E-4) return 0.0;
        return beta.get(j, 0);
    }

    /**
     * Returns the coefficient of determination <em>R</em><sup>2</sup>. (0-1)
     */
    public double R2() {
        if (sqaureSumTotal == 0.0) {
            return 1.0;
        }
        return 1.0 - squareSumError/sqaureSumTotal;
    }

    public int degree() {
        return degree;
    }

    /**
     * Returns the expected response y given the value of the predictor
     * variable x.
     */
    public double predict(double x) {
        // horner's method
        double y = 0.0;
        for (int j = degree; j >= 0; j--)
            y = beta(j) + (x * y);
        return y;
    }

    /** Lexiographic */
    public int compareTo(PolynomialRegression other) {
        double Epsilon = 1E-5;
        int maxDegree = Math.max(other.degree(), other.degree());
        for(int j = maxDegree; j >= 0; j--) {
            double term1 = 0.0;
            double term2 = 0.0;
            if(this.degree() >= j) {
                term1 = this.beta(j);
            }
            if(other.degree() >= j) {
                term2 = other.beta(j);
            }
            if(Math.abs(term1) < Epsilon) {
                term1 = 0.0;
            }
            if(Math.abs(term2) < Epsilon) {
                term2 = 0.0;
            }
            if(term1 < term2) {
                return -1;
            }
            else if(term1 > term2) {
                return +1;
            }
        }
        return 0;
    }
}
