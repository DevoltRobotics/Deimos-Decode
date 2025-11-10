package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class NextPosSorterCMD extends CommandBase {

    SpindexSubsystem spindexSubsystem;

    boolean reverse;
    public NextPosSorterCMD(SpindexSubsystem spindexSubsystem, boolean reverse){
        this.spindexSubsystem = spindexSubsystem;
        this.reverse = reverse;

        addRequirements(spindexSubsystem);
    }

    @Override
    public void initialize(){
        spindexSubsystem.nextPos(reverse);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
