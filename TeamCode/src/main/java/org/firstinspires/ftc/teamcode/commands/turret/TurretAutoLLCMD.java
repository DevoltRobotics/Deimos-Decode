package org.firstinspires.ftc.teamcode.commands.turret;

import com.acmerobotics.dashboard.FtcDashboard;
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


    public TurretAutoLLCMD(TurretSubsystem subsystem, LLSubsystem ll) {
        this.subsystem = subsystem;
        this.ll = ll;

        llPidf = new PIDFController(TurretSubsystem.llPidfCoeffs);

        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        llPidf.setCoefficients(TurretSubsystem.llPidfCoeffs);
        llPidf.setTolerance(0.1);
        llPidf.setMinimumOutput(TurretSubsystem.Minimum);

        if (ll.result != null && ll.result.isValid()) {
            Double Area = ll.getAllianceTA();
            if (Area != null) {

                if (Area > 0.55) {
                    offset = 0;
                } else if (Area < 0.55) {
                    offset = 3;
                }
            }

            llPidf.setSetPoint(0);

            Double tx = ll.getAllianceTX();


            if (ll.alliance == Alliance.BLUE ){
            if (tx != null) {
                double turretPower = llPidf.calculate(tx-offset);
                subsystem.setTurretPower(turretPower);
            } else {
                subsystem.setTurretPower(0);
                }
            } else if (ll.alliance == Alliance.RED) {

                if (tx != null) {
                    double turretPower = llPidf.calculate(tx+offset);
                    subsystem.setTurretPower(turretPower);
                } else {
                    subsystem.setTurretPower(0);
                }
            }
        }

    }
}
