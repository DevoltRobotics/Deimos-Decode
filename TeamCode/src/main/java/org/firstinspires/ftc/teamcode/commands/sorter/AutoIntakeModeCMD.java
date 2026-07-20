package org.firstinspires.ftc.teamcode.commands.sorter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class AutoIntakeModeCMD extends CommandBase {

    private final SpindexSubsystem spindex;

    // Estado interno del autoindex
    private boolean objectLatched = false;
    private double lastTriggerTimeMs = -9999;
    private final ElapsedTime timer = new ElapsedTime();




    public AutoIntakeModeCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        objectLatched = false;
        lastTriggerTimeMs = -9999;
        timer.reset();

        if(spindex.getFirstInitIn()) {
            spindex.setTargetPos(spindex.GetCloseIntakePos());
            spindex.setFirstInitIn(false);
            spindex.setFirstInitsho(true);
        }

        spindex.setShootmode(false);

    }

    @Override
    public void execute() {
        double nowMs = timer.milliseconds();



        if (spindex.getBPresence() && !objectLatched && !spindex.getShootmode() && spindex.StabaleTarget) {
             if (spindex.getDetectedColor() == SpindexSubsystem.DetectedColor.Green){
                 spindex.GrenBallPos = (spindex.getTargetPos() + 240);
             }
            spindex.advanceOneSorting();
            spindex.addnBalls();

            objectLatched = true;
            lastTriggerTimeMs = nowMs;
            if (spindex.getnBalls() == 3 && spindex.GrenBallPos < spindex.getTargetPos()){
                spindex.GrenBallPos = (spindex.getTargetPos() + 240);
            }
        }

        double dt = nowMs - lastTriggerTimeMs;

        if (!spindex.getBPresence() && dt > SpindexSubsystem.TRIGGER_COOLDOWN_MS && spindex.error < 20) {
            objectLatched = false;
        }
    }

    @Override
    public boolean isFinished() {
        return spindex.getnBalls() == 3;
    }
}
