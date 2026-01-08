package org.firstinspires.ftc.teamcode.commands.turret;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretToPosCMD extends CommandBase {

    PIDFController pidf;

    TurretSubsystem subsystem;

    protected Double targetPos;
    double error;

    boolean finishOnSetpoint = false;

    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos, boolean finishOnSetpoint) {
        this.subsystem = subsystem;

        pidf = new PIDFController(TurretSubsystem.encPidfCoeffs);
        this.targetPos = targetPos;

        this.finishOnSetpoint = finishOnSetpoint;

        addRequirements(subsystem);
    }

    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos) {
        this(subsystem, targetPos, true);
    }

    @Override
    public void execute() {

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

        subsystem.setTurretPower(pidf.calculate(error, 0));

        FtcDashboard.getInstance().getTelemetry().addData("turret target", targetPos);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }

    @Override
    public boolean isFinished() {
        if(finishOnSetpoint) {
            return Math.abs(pidf.getPositionError()) <= 5;
        } else {
            return false;
        }
    }
}