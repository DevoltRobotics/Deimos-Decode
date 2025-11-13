package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class ShootModeCMD extends InstantCommand {
    public ShootModeCMD(SpindexSubsystem spindex, HookSubsystem hook) {
        super(() -> {
            spindex.setNBalls(3);
            hook.nFlick = 0;
        });
    }
}
