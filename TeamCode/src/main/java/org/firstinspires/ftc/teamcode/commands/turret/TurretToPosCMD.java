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

    double targetPos;

    public TurretToPosCMD(TurretSubsystem subsystem, double targetPos) {
        this.subsystem = subsystem;

        pidf = new PIDFController(TurretSubsystem.encPidfCoeffs);
        this.targetPos = targetPos;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        pidf.setCoefficients(TurretSubsystem.encPidfCoeffs);
        pidf.setTolerance(1);
        pidf.setMinimumOutput(TurretSubsystem.Minimum);

        subsystem.setTurretPower(pidf.calculate(subsystem.getCurrentPosition(), targetPos));
    }

    @Override
    public boolean isFinished() {
        return pidf.atSetPoint();
    }
}
