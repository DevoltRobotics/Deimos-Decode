package org.firstinspires.ftc.teamcode.commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretToPosCMD extends CommandBase {

    PIDFController pidf;

    TurretSubsystem subsystem;

    Double targetPos;

    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos) {
        this.subsystem = subsystem;

        pidf = new PIDFController(TurretSubsystem.encPidfCoeffs);
        this.targetPos = targetPos;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if(targetPos == null) {
            cancel();
            return;
        }

        pidf.setCoefficients(TurretSubsystem.encPidfCoeffs);
        pidf.setTolerance(5);
        pidf.setMinimumOutput(TurretSubsystem.MinimumEnc);

        subsystem.setTurretPower(-pidf.calculate(subsystem.getCurrentPosition(), targetPos));

        FtcDashboard.getInstance().getTelemetry().addData("turret target", targetPos);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pidf.getPositionError()) <= 5;
    }
}
