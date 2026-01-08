package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.transfer.TransferCMD;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.HashMap;

public abstract class OpModeCommand extends OpMode {

    public IntakeSubsystem intakeSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public TurretSubsystem turretSubsystem;

    public SpindexSubsystem spindexSubsystem;
    public LLSubsystem llSubsystem;
    public HookSubsystem hookSubsystem;

    public PedroSubsystem pedroSubsystem;

    private Alliance alliance;

    public LiftSubsystem liftSubsystem;

    public TransferSubsystem transferSubsystem;

    private ArrayList<CommandLogEntry> logEntries = new ArrayList<>();

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().registerSubsystem(
                pedroSubsystem = new PedroSubsystem(hardwareMap),
                hookSubsystem = new HookSubsystem(hardwareMap),
                intakeSubsystem = new IntakeSubsystem(hardwareMap),
                shooterSubsystem = new ShooterSubsystem(hardwareMap),
                turretSubsystem = new TurretSubsystem(hardwareMap,pedroSubsystem, alliance),
                spindexSubsystem = new SpindexSubsystem(hardwareMap),
                llSubsystem = new LLSubsystem(hardwareMap, alliance),
                liftSubsystem = new LiftSubsystem(hardwareMap),
                transferSubsystem = new TransferSubsystem(hardwareMap)

        );

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        CommandScheduler.getInstance().onCommandInitialize((command) ->
                logEntries.add(new CommandLogEntry(3000, "Initialized", command.getName()))
        );
        CommandScheduler.getInstance().onCommandExecute((command) ->
                logEntries.add(new CommandLogEntry(command.hashCode(), -1, "Execute", command.getName()))
        );
        CommandScheduler.getInstance().onCommandFinish((command) ->
                logEntries.add(new CommandLogEntry(3000, "Finished", command.getName()))
        );
        CommandScheduler.getInstance().onCommandInterrupt((command) ->
                logEntries.add(new CommandLogEntry(3000, "Interrupted", command.getName()))
        );

        initialize();
    }

    @Override
    public final void loop() {
        CommandScheduler.getInstance().run();
        run();

        for (CommandLogEntry entry : logEntries) {
            // telemetry.addData("#" + entry.id + "[" + entry.telemetryCaption + "]", entry.telemetryMessage);
        }

        logEntries.removeIf((entry) -> entry.timer.milliseconds() >= entry.durationMillis);
    }

    public abstract void initialize();

    protected void log(int id, long durationMillis, String entry, String message) {
        logEntries.add(new CommandLogEntry(id, durationMillis, entry, message));
    }

    protected void log(long durationMillis, String entry, String message) {
        logEntries.add(new CommandLogEntry(durationMillis, entry, message));
    }


    static int entryCount;
    static HashMap<Integer, Integer> hashCodeMapping = new HashMap<>();

    static class CommandLogEntry {

        int id;
        ElapsedTime timer = new ElapsedTime();
        long durationMillis;

        String telemetryCaption;
        String telemetryMessage;

        CommandLogEntry(int hashCode, long durationMillis, String telemetryCaption, String telemetryMessage) {
            this.id = hashCodeMapping.computeIfAbsent(hashCode, ignored -> entryCount++);

            this.durationMillis = durationMillis;
            this.telemetryCaption = telemetryCaption;
            this.telemetryMessage = telemetryMessage;
        }

        CommandLogEntry(long durationMillis, String telemetryCaption, String telemetryMessage) {
            this.id = entryCount++;

            this.durationMillis = durationMillis;
            this.telemetryCaption = telemetryCaption;
            this.telemetryMessage = telemetryMessage;
        }
    }

    @Override
    public void stop(){

    }

}
