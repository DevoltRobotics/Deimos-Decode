package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class LoggedSubsystem extends SubsystemBase {
    public abstract void log(TelemetryPacket packet);
}