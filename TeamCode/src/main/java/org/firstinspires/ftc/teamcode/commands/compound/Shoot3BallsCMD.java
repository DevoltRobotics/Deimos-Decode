package org.firstinspires.ftc.teamcode.commands.compound;


import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexPosCMD;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class Shoot3BallsCMD extends SequentialCommandGroup {
    public Shoot3BallsCMD(HookSubsystem hookSubsystem, SpindexSubsystem spindexSubsystem, Double startingSpindex) {
        if(startingSpindex != null) {
            addCommands(new SpindexPosCMD(spindexSubsystem, startingSpindex));
        } else {
            addCommands(new ShootModeCMD(spindexSubsystem));
        }

        addCommands(
                new WaitCommand(400),
                new UpAndDownCMD(hookSubsystem,spindexSubsystem),
                new NextPosSorterCMD(spindexSubsystem),
                new WaitCommand(200),
                new UpAndDownCMD(hookSubsystem,spindexSubsystem),
                new NextPosSorterCMD(spindexSubsystem),
                new WaitCommand(200),
                new UpAndDownCMD(hookSubsystem,spindexSubsystem)
        );
    }

    public Shoot3BallsCMD(HookSubsystem hookSubsystem, SpindexSubsystem spindexSubsystem) {
        this(hookSubsystem, spindexSubsystem, null);
    }
}
