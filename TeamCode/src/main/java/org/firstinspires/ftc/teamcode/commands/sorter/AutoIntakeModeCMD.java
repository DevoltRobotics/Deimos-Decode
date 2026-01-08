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
    private Double lastTriggerPosRad = null;
    private final ElapsedTime timer = new ElapsedTime();



    public AutoIntakeModeCMD(SpindexSubsystem spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        spindex.detectedColor = SpindexSubsystem.DetectedColor.Unknown;
        objectLatched = false;
        lastTriggerTimeMs = -9999;
        lastTriggerPosRad = spindex.getTargetPos();
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
        double pos = spindex.getTargetPos();

        boolean Bpresence = spindex.getBPresence();

        //bola detectada
        if (Bpresence && !objectLatched && !spindex.getShootmode()) {
             spindex.detectedColor = spindex.getDetectedColor();
             if (spindex.detectedColor == SpindexSubsystem.DetectedColor.Green){
                 spindex.GrenBallPos = (spindex.getTargetPos() + 180);
             }
            spindex.advanceOneIndex();
            spindex.addnBalls();

            objectLatched = true;
            lastTriggerTimeMs = nowMs;
            lastTriggerPosRad = pos;
        }

        double dt = nowMs - lastTriggerTimeMs;

        FtcDashboard.getInstance().getTelemetry().addData("lastTriggerPosRad", lastTriggerPosRad);

        if (!Bpresence
                && dt > SpindexSubsystem.TRIGGER_COOLDOWN_MS
        ) {
            objectLatched = false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return spindex.getnBalls() == 3;
    }
}
