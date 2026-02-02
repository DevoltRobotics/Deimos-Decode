package org.firstinspires.ftc.teamcode.commands.sorter;

import com.acmerobotics.dashboard.FtcDashboard;
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
            spindex.setTargetPos(SpindexSubsystem.IntakePos);
            spindex.setFirstInitIn(false);
            spindex.setFirstInitsho(true);
        }

        spindex.setShootmode(false);

    }

    @Override
    public void execute() {
        double nowMs = timer.milliseconds();

        if (spindex.getBPresence() && !objectLatched && !spindex.getShootmode()) {
             if (spindex.sampleColorBestAlpha(8) == SpindexSubsystem.DetectedColor.Green){
                 spindex.GrenBallPos = (spindex.getTargetPos() + 180);
             }
            spindex.advanceOneIndex();
            spindex.addnBalls();

            objectLatched = true;
            lastTriggerTimeMs = nowMs;
        }

        double dt = nowMs - lastTriggerTimeMs;

        if (!spindex.getBPresence() && dt > SpindexSubsystem.TRIGGER_COOLDOWN_MS) {
            objectLatched = false;
        }
    }

    @Override
    public boolean isFinished() {
        return spindex.getnBalls() == 3;
    }
}
