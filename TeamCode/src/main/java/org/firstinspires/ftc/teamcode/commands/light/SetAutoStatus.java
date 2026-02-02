package org.firstinspires.ftc.teamcode.commands.light;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StatusLightSubsystem;

public class SetAutoStatus extends CommandBase {

    StatusLightSubsystem statusLightSubsystem;

    SpindexSubsystem spindexSubsystem;

    public SetAutoStatus(StatusLightSubsystem statusLightSubsystem,SpindexSubsystem spindexSubsystem){
        this.statusLightSubsystem = statusLightSubsystem;
        this.spindexSubsystem = spindexSubsystem;
        addRequirements(statusLightSubsystem);
    }

    @Override
    public void execute(){
        if (spindexSubsystem.getnBalls() == 3){
            statusLightSubsystem.SetGreenLight();
        } else if (spindexSubsystem.isShooting) {
            statusLightSubsystem.SetRedLight();
        }else{
            statusLightSubsystem.SetPurpleLight();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
