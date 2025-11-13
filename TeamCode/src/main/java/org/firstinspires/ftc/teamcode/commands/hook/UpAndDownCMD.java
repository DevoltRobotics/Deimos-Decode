package org.firstinspires.ftc.teamcode.commands.hook;


import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

public class UpAndDownCMD extends SequentialCommandGroup {

    public UpAndDownCMD(HookSubsystem hookSubsystem) {
        super(
                new HookUpCMD(hookSubsystem),
                new WaitCommand(300),
                new HookDownCMD(hookSubsystem),
                new WaitCommand(300),
                new InstantCommand(()->{
                    hookSubsystem.addnFLick();
                })
        );
    }

}
