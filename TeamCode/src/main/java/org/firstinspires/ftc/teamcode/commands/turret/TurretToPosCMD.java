package org.firstinspires.ftc.teamcode.commands.turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
public class TurretToPosCMD extends CommandBase {

    PIDFController pidf;
    PIDFController Secpidf;

    public static double filterAlpha = 0.9;
    private double filteredOutput = 0.0;

    public static double secondPidThreshold = 20;
    public static PIDFCoefficients Seccoeffs = new PIDFCoefficients(0.0001,0,0.000027,0);

    TurretSubsystem subsystem;

    protected Double targetPos;
    double error;

    double pidOut = 0;

    double SecpidOut;

    double ff = 0 ;

    public static double kS = 0.065;

    boolean finishOnSetpoint;

    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos, boolean finishOnSetpoint) {
        this.subsystem = subsystem;

        pidf = new PIDFController(TurretSubsystem.encPidfCoeffs);
        Secpidf = new PIDFController(Seccoeffs);
        this.targetPos = targetPos;

        this.finishOnSetpoint = finishOnSetpoint;

        addRequirements(subsystem);
    }

    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos) {
        this(subsystem, targetPos, true);
    }

    @Override
    public void execute() {

        Secpidf.setCoefficients(Seccoeffs);

        if(targetPos == null) {
            cancel();
            return;
        }

        double shortError = AngleUnit.normalizeDegrees(targetPos - TurretSubsystem.currentRelativePos);
        double longError = shortError > 0
                ? shortError - 360
                : shortError + 360;

        double predictedShort = TurretSubsystem.currentRelativePos + shortError;

        if(predictedShort >= TurretSubsystem.lowerLimit && predictedShort <= TurretSubsystem.upperLimit) {
            error = shortError;
        } else {
            error = longError;
        }

        pidf.setCoefficients(TurretSubsystem.encPidfCoeffs);
        pidf.setMinimumOutput(TurretSubsystem.MinimumEnc);

        pidOut = pidf.calculate(error,0);
        SecpidOut= applyExponentialFilter(Secpidf.calculate(error,0));

        ff = Math.signum(error) * kS;


        if (Math.abs(error)<1){
            subsystem.setTurretPower(0);
        } else if ((Math.abs(error) < secondPidThreshold) && (Math.abs(error)>1)) {
            subsystem.setTurretPower(SecpidOut-ff);
        } else {
            subsystem.setTurretPower(pidOut);

        }

        FtcDashboard.getInstance().getTelemetry().addData("turret target", targetPos);
        FtcDashboard.getInstance().getTelemetry().addData("turretFF", ff);
        FtcDashboard.getInstance().getTelemetry().addData("turretPID", pidOut);
        FtcDashboard.getInstance().getTelemetry().addData("turretError", error);

    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }

    @Override
    public boolean isFinished() {
        if(finishOnSetpoint) {
            return Math.abs(pidf.getPositionError()) <= 1;
        } else {
            return false;
        }
    }

    private double applyExponentialFilter(double input) {
        filteredOutput = filterAlpha * input + (1 - filterAlpha) * filteredOutput;
        return filteredOutput;
    }
}