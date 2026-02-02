package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.intakeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexModeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Autonomous(name = "Red-Cerca", group = "##", preselectTeleOp = "TeleOpRed")
public class RedA extends OpModeCommand {

    private Command autoCommand;


    public RedA() {
        super(Alliance.RED);
    }

    @Override
    public void initialize() {
        spindexSubsystem.setTargetPos(SpindexSubsystem.ShootPos);

        spindexSubsystem.SARSP();
        spindexSubsystem.setnBalls(3);

        Pose startPose = new Pose(123.90184049079755, 119.04294478527608, Math.toRadians(0));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();


        Pose ShootPos = new Pose(86.521, 83, Math.toRadians(0));

        Pose Pick1Cycle = new Pose(129, 80, 0);

        Pose P3rdCycle = new Pose(137, 57, 0);

        //Pose P4thCycle = new Pose(126, 30.7, 0);

        Pose P4thCycle = new Pose(87, 110, 0);


        PathChain FShoot = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                ShootPos
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain GoTo1Cycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPos,
                                Pick1Cycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain Shoot2Cycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(new BezierLine(
                        Pick1Cycle,
                        ShootPos
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain Pick3rdCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootPos,
                                new Pose(74.87116564417178, 50.79754601226993),
                                P3rdCycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        PathChain Shoot3rdCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                P3rdCycle,
                                ShootPos
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        PathChain Pick4thCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPos,
                                P4thCycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shooterSubsystem.setDefaultCommand(new ShooterShootCmd(shooterSubsystem, 1200));

        //TODO
        autoCommand =
                new ParallelCommandGroup(
                        new intakeDefaultCMD(intakeSubsystem,spindexSubsystem),
                new SequentialCommandGroup(
                new TurretToPosCMD(turretSubsystem,0d),
                new HookDownCMD(hookSubsystem),
                new ShootModeCMD(spindexSubsystem),

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(FShoot),
                        new TurretToPosCMD(turretSubsystem,-109.9)
                        ),

                new RunCommand(() ->
                        // eval obelisk here to store for the rest of the auto
                        SpindexSubsystem.obeliskPattern = llSubsystem.getObelisk()
                ).withTimeout(800),

                new TurretToPosCMD(turretSubsystem,-45d),



                new Shoot3BallsCMD(hookSubsystem,spindexSubsystem,()->spindexSubsystem.getPatternOffset()),


                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.3)),

                        new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(GoTo1Cycle),
                                new SpindexModeDefaultCMD(spindexSubsystem)
                        ),
                new WaitCommand(300),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Shoot2Cycle),
                        new SpindexModeDefaultCMD(spindexSubsystem)
                ),

                new Shoot3BallsCMD(hookSubsystem,spindexSubsystem, ()->spindexSubsystem.getPatternOffset()),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.4)),

                        new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Pick3rdCycle),
                        new SpindexModeDefaultCMD(spindexSubsystem),
                        new InstantCommand(()->spindexSubsystem.setFirstInitIn(true))
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Shoot3rdCycle),
                        new SpindexModeDefaultCMD(spindexSubsystem)
                ),

                new Shoot3BallsCMD(hookSubsystem,spindexSubsystem,()->spindexSubsystem.getPatternOffset()),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Pick4thCycle),
                        new TurretToPosCMD(turretSubsystem,0d),
                        new SpindexModeDefaultCMD(spindexSubsystem)
                )
        ));
    }

    @Override
    public void start() {
        if (autoCommand != null) {
            // Programamos el auton en el scheduler
            schedule(autoCommand);
        }
    }

    @Override
    public void run() {
        // Telemetría para debug
        Pose pose = pedroSubsystem.follower.getPose();
        PedroSubsystem.robotPose = pedroSubsystem.follower.getPose();

        telemetry.addData("patron",spindexSubsystem.obeliskPattern);
        telemetry.addData("Follower busy", pedroSubsystem.follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
    }
}

