package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class IntakeModeCMD extends InstantCommand {
    public IntakeModeCMD(SpindexSubsystem spindex, HookSubsystem hook) {
        super(() -> {
            spindex.setNBalls(-1);
            hook.nFlick = 3;
        });
    }
}
