package org.firstinspires.ftc.teamcode.commands.sorter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class SpindexPosCMD extends CommandBase {

    SpindexSubsystem spindexSubsystem;
    ElapsedTime timer = new ElapsedTime();

    double error;
    double pos;

    public SpindexPosCMD(SpindexSubsystem spindexSubsystem, double pos) {
        this.spindexSubsystem = spindexSubsystem;
        this.pos = pos;
        addRequirements(spindexSubsystem);
    }

    @Override
    public void initialize() {
        error = Math.abs(pos - spindexSubsystem.getTargetPos());
        spindexSubsystem.setTargetPos(pos);

        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > (error * SpindexSubsystem.SpindexDelayFactorSeconds);
    }

}
