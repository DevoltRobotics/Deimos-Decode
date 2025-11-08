package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AngularPIDFController {
    private PIDFCoefficients coeffs;
    private double setpointDeg = 0.0;

    // Integral & derivada
    private double totalError = 0.0;
    private Double prevError = null;
    private final ElapsedTime timer = new ElapsedTime();

    // Tunables/seguridad
    private double deadbandDeg = 0.0;                 // zona muerta en grados
    private double iZoneDeg = Double.POSITIVE_INFINITY; // integra solo dentro de este error
    private double iMaxAbs = Double.POSITIVE_INFINITY;  // clamp al término integral
    private double outMin = -1.0, outMax = 1.0;         // límites de salida
    private double ffTerm = 0.0;                        // término F opcional

    public AngularPIDFController(PIDFCoefficients coeffs) {
        this.coeffs = coeffs;
        timer.reset();
    }

    public void setCoefficients(PIDFCoefficients c){ this.coeffs = c; }
    public void setSetPointDegrees(double sp){ this.setpointDeg = wrap0to360(sp); }
    public void setDeadbandDeg(double d){ this.deadbandDeg = Math.max(0, d); }
    public void setIZoneDeg(double iz){ this.iZoneDeg = Math.max(0, iz); }
    public void setIMaxAbs(double i){ this.iMaxAbs = Math.max(0, i); }
    public void setOutputLimits(double min, double max){ this.outMin = Math.min(min,max); this.outMax = Math.max(min,max); }
    public void setFeedforward(double ff){ this.ffTerm = ff; }
    public void reset(){ totalError = 0; prevError = null; timer.reset(); }

    public double calculateDegrees(double measurementDeg){
        double dt = Math.max(1e-3, timer.seconds()); // s desde el último llamado
        timer.reset();

        double meas = wrap0to360(measurementDeg);
        double error = signedAngleDiff(setpointDeg, meas); // [-180,180)

        // Deadband
        if (Math.abs(error) <= deadbandDeg) {
            prevError = error;
            return clamp(0.0, outMin, outMax);
        }

        // P
        double p = coeffs.p * error;

        // I (con zona y clamp)
        if (Math.abs(error) <= iZoneDeg) {
            totalError += error * dt;
            if (Math.abs(totalError) > iMaxAbs) {
                totalError = Math.signum(totalError) * iMaxAbs;
            }
        } else {
            // opcional: “decay” suave
            totalError *= 0.9;
        }
        double i = coeffs.i * totalError;

        // D
        double d = 0.0;
        if (prevError != null) {
            double deriv = (error - prevError) / dt;
            d = coeffs.d * deriv;
        }
        prevError = error;

        double out = p + i + d + coeffs.f * ffTerm;
        return clamp(out, outMin, outMax);
    }

    // -------- helpers --------
    private static double wrap0to360(double a){ double x = a % 360.0; return (x < 0) ? x + 360.0 : x; }
    private static double signedAngleDiff(double target, double current){
        double diff = target - current;
        diff = (diff + 540.0) % 360.0 - 180.0; // -> [-180,180)
        return diff;
    }
    private static double clamp(double v, double lo, double hi){ return Math.max(lo, Math.min(hi, v)); }
}
