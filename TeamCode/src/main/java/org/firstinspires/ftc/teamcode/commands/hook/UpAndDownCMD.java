package org.firstinspires.ftc.teamcode.commands.hook;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class UpAndDownCMD extends SequentialCommandGroup {

    public UpAndDownCMD(HookSubsystem hookSubsystem, SpindexSubsystem spindexSubsystem) {
        super(
                new HookUpCMD(hookSubsystem),
                new WaitCommand(300),
                new HookDownCMD(hookSubsystem),
                new WaitCommand(300),
                new InstantCommand(()->{
                    spindexSubsystem.lessBalls();
                })
        );
    }

}
