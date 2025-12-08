package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class IntakeModeCMD extends CommandBase {

    private final SpindexSubsystem spindex;
    private boolean done = false;

    public IntakeModeCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        spindex.setTargetPos(spindex.IntakePos);
        done = false;
        spindex.setShootmode(false);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
