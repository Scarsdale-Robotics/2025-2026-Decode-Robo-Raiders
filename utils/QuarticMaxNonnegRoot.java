package org.firstinspires.ftc.teamcode.utils;

// Credit chatgpt
public final class QuarticMaxNonnegRoot {
    // Public API: coeffs are in descending order: a_n, a_{n-1}, ..., a0
    public static double maxNonNegativeRoot(double[] coeffs) {
        // Trim leading zeros and handle trivial cases
        coeffs = trimLeadingZeros(coeffs);
        int n = coeffs.length - 1;
        if (n < 1) return Double.NaN; // constant polynomial -> no roots

        // Normalize so leading coefficient = 1
        double lead = coeffs[0];
        for (int i = 0; i < coeffs.length; i++) coeffs[i] /= lead;

        // Find complex roots using Durand-Kerner
        Complex[] roots = durandKerner(coeffs, 2000, 1e-14);

        // Pick real roots (small imaginary part)
        double imagTol = 1e-8;
        double negTol = -1e-12; // consider tiny negatives as zero
        double best = Double.NaN;
        for (Complex z : roots) {
            if (Math.abs(z.im) <= imagTol) {
                double r = z.re;
                if (r < 0 && r > negTol) r = 0.0; // snap tiny negatives
                if (r >= 0.0) {
                    if (Double.isNaN(best) || r > best) best = r;
                }
            }
        }
        return best;
    }

    // --- Durand-Kerner implementation ---
    private static Complex[] durandKerner(double[] a, int maxIter, double tol) {
        int n = a.length - 1;
        Complex[] x = new Complex[n];
        // initial radius R = 1 + max |a_k|
        double R = 0;
        for (int i = 1; i < a.length; i++) R = Math.max(R, Math.abs(a[i]));
        R = 1 + R;
        // initialize points on circle of radius R
        for (int i = 0; i < n; i++) {
            double theta = 2 * Math.PI * i / n;
            x[i] = new Complex(R * Math.cos(theta), R * Math.sin(theta));
        }

        for (int iter = 0; iter < maxIter; iter++) {
            boolean converged = true;
            for (int i = 0; i < n; i++) {
                Complex xi = x[i];
                Complex P = evalPoly(a, xi);
                Complex denom = new Complex(1, 0);
                for (int j = 0; j < n; j++) {
                    if (i == j) continue;
                    Complex diff = xi.sub(x[j]);
                    // avoid exact zero denom
                    if (diff.abs() == 0.0) diff = diff.add(new Complex(1e-16, 1e-16));
                    denom = denom.mul(diff);
                }
                Complex delta = P.div(denom);
                x[i] = xi.sub(delta);
                if (delta.abs() > tol) converged = false;
            }
            if (converged) break;
        }
        return x;
    }

    // Evaluate polynomial with normalized coefficients (leading coeff = 1)
    private static Complex evalPoly(double[] a, Complex z) {
        Complex res = new Complex(a[0], 0); // leading (should be 1)
        for (int i = 1; i < a.length; i++) {
            res = res.mul(z).add(new Complex(a[i], 0));
        }
        return res;
    }

    // Trim leading zeros in coefficient array
    private static double[] trimLeadingZeros(double[] c) {
        int i = 0;
        while (i < c.length && Math.abs(c[i]) == 0.0) i++;
        if (i == 0) return c.clone();
        if (i >= c.length) return new double[]{0.0}; // zero polynomial
        double[] out = new double[c.length - i];
        System.arraycopy(c, i, out, 0, out.length);
        return out;
    }

    // Simple complex number helper
    private static final class Complex {
        final double re, im;
        Complex(double r, double i) { re = r; im = i; }
        Complex add(Complex o) { return new Complex(re + o.re, im + o.im); }
        Complex sub(Complex o) { return new Complex(re - o.re, im - o.im); }
        Complex mul(Complex o) { return new Complex(re * o.re - im * o.im, re * o.im + im * o.re); }
        Complex div(Complex o) {
            double denom = o.re*o.re + o.im*o.im;
            return new Complex((re*o.re + im*o.im)/denom, (im*o.re - re*o.im)/denom);
        }
        double abs() { return Math.hypot(re, im); }
        Complex add(double d) { return new Complex(re + d, im); }
    }

    // Example usage
    public static void main(String[] args) {
        // Example: x^4 - 5x^3 + 8x^2 - 4x = 0
        double[] coeffs = {1, -5, 8, -4, 0};
        double root = maxNonNegativeRoot(coeffs);
        if (Double.isNaN(root)) System.out.println("No nonnegative real root");
        else System.out.println("Max nonnegative root = " + root);
    }
}
