package org.firstinspires.ftc.teamcode.commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config

public class TurretAutoLLCMD extends CommandBase {


    PIDFController llPidf;

    TurretSubsystem subsystem;
    LLSubsystem ll;

    double offset = 0;

    MovingStatistics llAverage = new MovingStatistics(5);

    public static org.firstinspires.ftc.teamcode.config.PIDFController.PIDCoefficients turretCoeffs = new org.firstinspires.ftc.teamcode.config.PIDFController.PIDCoefficients(
            0.003, 0, 0
    );

    public org.firstinspires.ftc.teamcode.config.PIDFController TurretController = new org.firstinspires.ftc.teamcode.config.PIDFController(turretCoeffs);

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

                double parallaxDeg = Math.toDegrees(Math.atan2(TurretSubsystem.LL_OFFSET, subsystem.distanceToGoal));

                double mean = llAverage.getMean();

                FtcDashboard.getInstance().getTelemetry().addData("ll tX mean", mean);
                FtcDashboard.getInstance().getTelemetry().addData("ll tX parallax", parallaxDeg);

                TurretController.targetPosition = 0;
                double power = TurretController.update(ll.getAllianceTX());
                double turretPower = llPidf.calculate(mean - parallaxDeg, 0);

                subsystem.setTurretPower(power);
            } else {
                subsystem.setTurretPower(0);
            }
        } else {
            subsystem.setTurretPower(0);
        }
        FtcDashboard.getInstance().getTelemetry().addData("LL error",llPidf.getPositionError() );
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.setTurretPower(0);
    }
}
