package org.firstinspires.ftc.teamcode.commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretAutoLLCMD extends CommandBase {


    PIDFController llPidf;

    TurretSubsystem subsystem;
    LLSubsystem ll;


    public TurretAutoLLCMD(TurretSubsystem subsystem, LLSubsystem ll) {
        this.subsystem = subsystem;
        this.ll = ll;

        llPidf = new PIDFController(TurretSubsystem.llPidfCoeffs);

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        llPidf.setCoefficients(TurretSubsystem.llPidfCoeffs);
        llPidf.setSetPoint(0);
        llPidf.setTolerance(0.1);
        llPidf.setMinimumOutput(TurretSubsystem.Minimum);

        Double tx = ll.getAllianceTX();

        if(tx != null) {
            double turretPower = llPidf.calculate(tx);
            subsystem.setTurretPower(turretPower);

            FtcDashboard.getInstance().getTelemetry().addData("turret power", turretPower);
        } else {
            subsystem.setTurretPower(0);
        }
    }
}
