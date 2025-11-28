package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class SpinSorterCMD extends CommandBase {

    SpindexSubsystem spindexSubsystem;

    double targetPos;

    public SpinSorterCMD(SpindexSubsystem spindexSubsystem, double targetPos){
        this.spindexSubsystem = spindexSubsystem;
        this.targetPos = AngleUnit.normalizeRadians(targetPos);

        addRequirements(spindexSubsystem);
    }

    @Override
    public void initialize(){
        spindexSubsystem.manualMode = true;
        spindexSubsystem.setTargetPos(targetPos);
    }


    @Override
    public boolean isFinished(){
        return spindexSubsystem.isAtTarget();
    }

    @Override
    public void end(boolean interrupted){
        spindexSubsystem.manualMode = false;
    }

}
