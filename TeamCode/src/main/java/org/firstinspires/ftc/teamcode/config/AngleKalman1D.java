package org.firstinspires.ftc.teamcode.config;

public class AngleKalman1D {
    private double xDeg;     // estimated angle
    private double P;        // covariance
    private double Q;        // process noise
    private double R;        // measurement noise (for LL)
    private Double lastOdoDeg = null;

    public AngleKalman1D(double x0Deg, double P0, double Q, double R) {
        this.xDeg = x0Deg;
        this.P = P0;
        this.Q = Q;
        this.R = R;
        this.lastOdoDeg = null;
    }

    public void setQ(double Q) { this.Q = Q; }
    public void setR(double R) { this.R = R; }

    public void reset(double x0Deg) {
        xDeg = normalizeDeg(x0Deg);
        P = 1.0;
        lastOdoDeg = null;
    }

    /** Predict step using odometry angle changes (so it tracks smoothly). */
    public void predictFromOdo(double odoDesiredDeg) {
        odoDesiredDeg = normalizeDeg(odoDesiredDeg);

        if (lastOdoDeg == null) {
            lastOdoDeg = odoDesiredDeg;
            xDeg = odoDesiredDeg;   // initialize to odo on first run
            return;
        }

        double dOdo = angleErrorDeg(odoDesiredDeg, lastOdoDeg); // shortest delta
        xDeg = normalizeDeg(xDeg + dOdo);
        lastOdoDeg = odoDesiredDeg;

        P = P + Q;
    }

    /** Update step with limelight measurement, if available. */
    public void updateWithLL(double llDesiredDeg) {
        llDesiredDeg = normalizeDeg(llDesiredDeg);

        double y = angleErrorDeg(llDesiredDeg, xDeg);   // innovation (shortest)
        double S = P + R;
        double K = P / S;

        xDeg = normalizeDeg(xDeg + K * y);
        P = (1.0 - K) * P;
    }

    public double get() { return xDeg; }

    // --- angle helpers ---
    public static double normalizeDeg(double deg) {
        deg = deg % 360.0;
        if (deg <= -180) deg += 360;
        if (deg > 180) deg -= 360;
        return deg;
    }

    /** returns (target - current) in [-180, 180] */
    public static double angleErrorDeg(double target, double current) {
        return normalizeDeg(target - current);
    }
}
