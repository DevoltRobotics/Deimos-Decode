package org.firstinspires.ftc.teamcode.commands.turret;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;


public class TurretAutoFuseCMD extends CommandBase {

    private final TurretSubsystem turret;
    private final LLSubsystem ll;

    private ElapsedTime llTimeoutTimer = new ElapsedTime();
    private ElapsedTime odoTimeoutTimer = new ElapsedTime();

    Command activeCommand;

    public TurretAutoFuseCMD(TurretSubsystem subsystem, LLSubsystem llSubsystem) {
        this.turret = subsystem;
        this.ll = llSubsystem;

        activeCommand = new TurretAutoOdoCMD(subsystem);

        // require both so nothing else fights LL pipeline / turret
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        activeCommand.initialize();
    }

    @Override
    public void execute() {
        if(ll.getAllianceTX() != null) {
            llTimeoutTimer.reset();
        }

        if(ll.getAllianceTX() == null && !(activeCommand instanceof TurretAutoOdoCMD) && llTimeoutTimer.seconds() > 2) {
            activeCommand = new TurretAutoOdoCMD(turret);
            odoTimeoutTimer.reset();
            activeCommand.initialize();
        } else if(ll.getAllianceTX() != null && !(activeCommand instanceof TurretAutoLLCMD) && odoTimeoutTimer.seconds() > 2){
            activeCommand = new TurretAutoLLCMD(turret, ll);
            activeCommand.initialize();
        }

        activeCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        activeCommand.end(interrupted);
    }
}
