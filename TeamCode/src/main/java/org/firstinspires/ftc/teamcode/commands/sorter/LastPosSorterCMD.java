package org.firstinspires.ftc.teamcode.commands.sorter;

import com.qualcomm.robotcore.util.ElapsedTime;
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
        spindex.returnOneIndex();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(spindex.SpinPID.getPositionError()) <= 5;
    }


}
