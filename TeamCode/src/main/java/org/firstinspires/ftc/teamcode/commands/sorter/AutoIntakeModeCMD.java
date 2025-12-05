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
        // Opcional: resetear estado
        objectLatched = false;
        lastTriggerTimeMs = -9999;
        lastTriggerPosRad = spindex.getSpindexPos();
        timer.reset();

        if(spindex.getFirstInitIn()) {
            spindex.setTargetPosRad(spindex.IntakePos);
            spindex.setFirstInitIn(false);
            spindex.setFirstInitsho(true);
        }

        spindex.setShootmode(false);

    }

    @Override
    public void execute() {
        double nowMs = timer.milliseconds();
        double pos = spindex.getSpindexPos();

        boolean Bpresence = spindex.getBPresence();

        //bola detectada
        if (Bpresence && !objectLatched && !spindex.getShootmode()) {
            spindex.advanceOneIndex();
            spindex.addnBalls();

            objectLatched = true;
            lastTriggerTimeMs = nowMs;
            lastTriggerPosRad = pos;
        }

        double dt = nowMs - lastTriggerTimeMs;
        double dAngle = spindex.getAngleDiff(pos, lastTriggerPosRad);

        FtcDashboard.getInstance().getTelemetry().addData("dAngle", dAngle);
        FtcDashboard.getInstance().getTelemetry().addData("lastTriggerPosRad", lastTriggerPosRad);

        if (!Bpresence
                && dt > SpindexSubsystem.TRIGGER_COOLDOWN_MS
                && dAngle >= SpindexSubsystem.MIN_ADVANCE_RAD
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
