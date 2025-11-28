package org.firstinspires.ftc.teamcode.subsystems;

public class QuadraticTheta {

    // {dist (in), angle (radans)}
    private final double[][] data;

    // degree (quad for now)
    private final int degree;

    // poly coefficants
    public double[] coeffs;

    public QuadraticTheta(double[][] Data) {
        this.data = Data;
        this.degree = 2; // change
        coeffs = fitPolynomial(data, degree);
    }



    public static double[] fitPolynomial(double[][] data, int degree) {
        int n = data.length;
        int m = degree + 1;

        double[][] X = new double[n][m];
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            double distance = data[i][0];
            y[i] = data[i][1];
            double pow = 1.0;
            for (int j = 0; j < m; j++) {
                X[i][j] = pow;
                pow *= distance;
            }
        }

        return NormalEquation(X, y);
    }


    // this is from stevens math i dont know how it works but should supposedly give us the normal equation for each point so gaussian can solve it
    private static double[] NormalEquation(double[][] X, double[] y) {
        int n = X.length;
        int m = X[0].length;

        double[][] XtX = new double[m][m];
        double[] Xty = new double[m];

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                Xty[j] += X[i][j] * y[i];
                for (int k = 0; k < m; k++) {
                    XtX[j][k] += X[i][j] * X[i][k];
                }
            }
        }
        return gaussianElimination(XtX, Xty);
    }



    //solves normal equations (:))
    private static double[] gaussianElimination(double[][] A, double[] b) {
        int n = b.length;
        for (int i = 0; i < n; i++) {
            int max = i;
            for (int j = i + 1; j < n; j++)
                if (Math.abs(A[j][i]) > Math.abs(A[max][i])) max = j;

            double[] tempRow = A[i]; A[i] = A[max]; A[max] = tempRow;
            double tempVal = b[i]; b[i] = b[max]; b[max] = tempVal;

            for (int j = i + 1; j < n; j++) {
                double factor = A[j][i] / A[i][i];
                b[j] -= factor * b[i];
                for (int k = i; k < n; k++) A[j][k] -= factor * A[i][k];
            }
        }

        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = b[i];
            for (int j = i + 1; j < n; j++) sum -= A[i][j] * x[j];
            x[i] = sum / A[i][i];
        }

        return x;
    }



    //returns theta using equation given dis
    public double evaluate(double dist) {
        double theta = 0;
        for (int i = 0; i < coeffs.length; i++) {
            theta += coeffs[i] * Math.pow(dist, i);
        }
        return theta;
    }


    //returns residuals given data and coeffs
    public static double[][] calculateResiduals(double[][] data, double[] coeffs) {
        int n = data.length;
        double[][] residuals = new double[n][2]; // [dist, e]

        for (int i = 0; i < n; i++) {
            double distance = data[i][0];
            double actualTheta = data[i][1];

            double predictedTheta = 0;
            for (int j = 0; j < coeffs.length; j++) {
                predictedTheta += coeffs[j] * Math.pow(distance, j);
            }

            residuals[i][0] = distance;
            residuals[i][1] = actualTheta - predictedTheta;
        }

        return residuals;
    }

    public static double calculateRMSE(double[][] data, double[] coeffs) {
        double[][] residuals = calculateResiduals(data, coeffs);
        int n = residuals.length;
        double sumSquared = 0;

        for (int i = 0; i < n; i++) {
            double residual = residuals[i][1];
            sumSquared += residual * residual;
        }

        return Math.sqrt(sumSquared / n);
    }

}
