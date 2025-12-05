package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class SpindexModeDefaultCMD extends CommandBase {
    private final SpindexSubsystem spindex;
    private final Command shootCmd;
    private final Command autoCmd;

    private Command current = null;

    public SpindexModeDefaultCMD(SpindexSubsystem spindex, Command shootCmd, Command autoCmd) {
        this.spindex = spindex;
        this.shootCmd = shootCmd;
        this.autoCmd = autoCmd;
        addRequirements(spindex);
    }

    @Override public void initialize() {
        current = spindex.getShootmode() ? shootCmd : autoCmd;
        current.initialize();
    }

    @Override public void execute() {
        Command desired = spindex.getShootmode() ? shootCmd : autoCmd;

        if (desired != current) {
            if (current != null) current.end(true);
            current = desired;
            current.initialize();
        }
        current.execute();
    }

    @Override public void end(boolean interrupted) {
        if (current != null) current.end(interrupted);
    }

    @Override public boolean isFinished() { return false; }
}
