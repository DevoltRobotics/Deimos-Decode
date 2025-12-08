package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LLSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

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

    public OpModeCommand(Alliance alliance) {
        this.alliance = alliance;
    }

    //reinicia la lista de comandos
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    //corre el scheduler
    public void run() {
        CommandScheduler.getInstance().run();
    }

    //programa comandos al scheduler
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    //registra subsistemas al scheduler
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        register(
                hookSubsystem = new HookSubsystem(hardwareMap),
                intakeSubsystem = new IntakeSubsystem(hardwareMap),
                shooterSubsystem = new ShooterSubsystem(hardwareMap),
                turretSubsystem = new TurretSubsystem(hardwareMap),
                spindexSubsystem = new SpindexSubsystem(hardwareMap),
                llSubsystem = new LLSubsystem(hardwareMap, alliance),
                pedroSubsystem = new PedroSubsystem(hardwareMap),
                liftSubsystem = new LiftSubsystem(hardwareMap)
        );


        initialize();
    }




    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        run();
    }

    public void stop() {
        reset();
    }

    public abstract void initialize();


}
