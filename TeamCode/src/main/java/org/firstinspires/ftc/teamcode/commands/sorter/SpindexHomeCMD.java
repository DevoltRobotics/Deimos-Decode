package org.firstinspires.ftc.teamcode.commands.sorter;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class SpindexHomeCMD extends CommandBase {

    SpindexSubsystem subsystem;

    public SpindexHomeCMD(SpindexSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        if(!subsystem.isAtHome()) {
            subsystem.setTargetPos(subsystem.getTargetPos() + 0.5); // incrementally update until we get to home
        } else {
            subsystem.resetRelativePos(); // reset encoder once we're at home
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.isAtHome();
    }
}
