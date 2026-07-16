package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class LastPosSorterCMD extends CommandBase {
    private final SpindexSubsystem spindex;



    public LastPosSorterCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        spindex.advanceOneshooting();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(spindex.SpinPID.getPositionError()) <= 5;
    }


}
