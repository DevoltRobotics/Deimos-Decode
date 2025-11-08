package org.firstinspires.ftc.teamcode.commands.hook;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

public class HookUpCMD extends CommandBase {
    HookSubsystem subsystem;

    public HookUpCMD(HookSubsystem subsystem){

        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        subsystem.hookUp();
    }

    @Override
    public boolean isFinished(){
        return subsystem.hook.getPosition() >= 0.199;
    }
}
