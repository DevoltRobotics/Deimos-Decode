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

    TurretSubsystem subsystem;

    protected Double targetPos;
    double Realtarget;



    public TurretToPosCMD(TurretSubsystem subsystem, Double targetPos) {
        this.subsystem = subsystem;

        this.targetPos = targetPos;


        addRequirements(subsystem);
    }



    @Override
    public void execute() {


        if(targetPos == null) {
            cancel();
            return;
        }

        if ( targetPos > TurretSubsystem.upperLimit){
          Realtarget =  targetPos - 360;
        } else if (targetPos < TurretSubsystem.lowerLimit) {
            Realtarget = targetPos + 360;
        }else {
            Realtarget = targetPos;
        }


        subsystem.TurretSetPos(Realtarget);


        FtcDashboard.getInstance().getTelemetry().addData("turret target", Realtarget);

    }



    @Override
    public boolean isFinished() {
            return true;
    }


}