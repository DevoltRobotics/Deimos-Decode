package org.firstinspires.ftc.teamcode.commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretAutoLLCMD extends CommandBase {


    PIDFController llPidf;

    TurretSubsystem subsystem;
    LLSubsystem ll;

    double offset = 0;

    MovingStatistics llAverage = new MovingStatistics(5);

    public TurretAutoLLCMD(TurretSubsystem subsystem, LLSubsystem ll) {
        this.subsystem = subsystem;
        this.ll = ll;

        llPidf = new PIDFController(TurretSubsystem.llPidfCoeffs);

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ll.setAimingPipeline();

        llPidf.setCoefficients(TurretSubsystem.llPidfCoeffs);
        llPidf.setTolerance(0.1);
        llPidf.setMinimumOutput(TurretSubsystem.Minimum);

        if (ll.result != null && ll.result.isValid()) {
            Double Area = ll.getAllianceTA();

            if (Area != null) {
                if (Area > 0.55) {
                    offset = 0;
                } else if (Area < 0.55) {
                    offset = 2.8;
                }
            }

            Double tx = ll.getAllianceTX();

            if(tx != null) {
                double txOffset = ll.alliance == Alliance.BLUE ? tx - offset : tx + offset;
                llAverage.add(txOffset);

                double mean = llAverage.getMean();

                FtcDashboard.getInstance().getTelemetry().addData("ll tX mean", mean);

                double turretPower = llPidf.calculate(mean, 0);

                subsystem.setTurretPower(turretPower);
            } else {
                subsystem.setTurretPower(0);
            }
        } else {
            subsystem.setTurretPower(0);
        }
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }
}
