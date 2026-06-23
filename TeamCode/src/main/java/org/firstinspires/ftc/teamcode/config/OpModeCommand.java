package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.StatusLightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class OpModeCommand extends OpMode {

    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public TurretSubsystem turretSubsystem;

    public SpindexSubsystem spindexSubsystem;
    public LLSubsystem llSubsystem;
    public HookSubsystem hookSubsystem;

    public PedroSubsystem pedroSubsystem;

    public final Alliance alliance;

    public LiftSubsystem liftSubsystem;


    public StatusLightSubsystem statusLightSubsystem;

    private List<Subsystem> subsystems;

    public OpModeCommand(Alliance alliance) {
        this.alliance = alliance;
    }

    //corre el scheduler
    public void run() {
    }

    //programa comandos al scheduler
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        subsystems = List.of(
                pedroSubsystem = new PedroSubsystem(hardwareMap, alliance),
                hookSubsystem = new HookSubsystem(hardwareMap),
                intakeSubsystem = new IntakeSubsystem(hardwareMap),
                shooterSubsystem = new ShooterSubsystem(hardwareMap),
                turretSubsystem = new TurretSubsystem(hardwareMap,pedroSubsystem, alliance),
                spindexSubsystem = new SpindexSubsystem(hardwareMap),
                llSubsystem = new LLSubsystem(hardwareMap, alliance),
                liftSubsystem = new LiftSubsystem(hardwareMap),
                statusLightSubsystem = new StatusLightSubsystem(hardwareMap)

        );

        for(Subsystem subsystem : subsystems) {
            CommandScheduler.getInstance().registerSubsystem(subsystem);
        }

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        initialize();
    }

    @Override
    public final void loop() {
        CommandScheduler.getInstance().run();
        run();


        TelemetryPacket packet = new TelemetryPacket();

        for(Subsystem subsystem : subsystems) {
            if(subsystem instanceof LoggedSubsystem) {
                ((LoggedSubsystem)subsystem).log(packet);
            }
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    public abstract void initialize();




}
