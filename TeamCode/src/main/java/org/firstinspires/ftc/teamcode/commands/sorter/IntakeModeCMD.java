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
        spindex.setTargetPosRad(spindex.IntakePos);
        done = false;
        spindex.setShootmode(false);
    }

    @Override
    public void execute() {
        if (spindex.getAngleError() < Math.toRadians(3)) {  // tolerancia angular
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }


}
