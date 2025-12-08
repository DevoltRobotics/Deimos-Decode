package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class ShootModeCMD extends CommandBase {

    private final SpindexSubsystem spindex;

    public ShootModeCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        if(spindex.getFirstInitSho()) {
            spindex.setTargetPos(spindex.ShootPos);
            spindex.setFirstInitsho(false);
            spindex.setFirstInitIn(true);
        }
        spindex.setShootmode(true);
    }



    @Override
    public boolean isFinished() {
        return true;
    }


}
