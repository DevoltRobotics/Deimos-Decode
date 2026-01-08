package org.firstinspires.ftc.teamcode.commands.sorter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

import java.util.function.DoubleSupplier;

public class SpindexPosCMD extends CommandBase {

    SpindexSubsystem spindexSubsystem;
    DoubleSupplier position;

    public SpindexPosCMD(SpindexSubsystem spindexSubsystem, DoubleSupplier pos) {
        this.spindexSubsystem = spindexSubsystem;
        this.position = pos;
        addRequirements(spindexSubsystem);
    }

    public SpindexPosCMD(SpindexSubsystem spindexSubsystem, double pos) {
        this(spindexSubsystem, () -> pos);
    }

    @Override
    public void initialize() {
        double pos = this.position.getAsDouble();
        spindexSubsystem.setTargetPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
