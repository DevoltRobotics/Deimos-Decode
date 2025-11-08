package org.firstinspires.ftc.teamcode.commands.hook;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

public class HookDownCMD extends CommandBase {
    HookSubsystem subsystem;

    public HookDownCMD(HookSubsystem subsystem){

        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.hookDown();
    }

    @Override
    public boolean isFinished(){
        return subsystem.hook.getPosition() <= 0.1;
    }
}
