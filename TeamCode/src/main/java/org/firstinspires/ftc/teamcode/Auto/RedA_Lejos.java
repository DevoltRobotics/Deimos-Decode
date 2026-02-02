package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
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

@Autonomous(name = "Red_Lejos", group = "##", preselectTeleOp = "TeleOpRed")
public class RedA_Lejos extends OpModeCommand {

    private Command autoCommand;



    public RedA_Lejos() {
        super(Alliance.RED);
    }

    @Override
    public void initialize() {
        spindexSubsystem.setTargetPos(SpindexSubsystem.ShootPos);
        spindexSubsystem.SARSP();
        spindexSubsystem.setnBalls(3);
        Pose startPose = new Pose(80.58715596330276, 8.807339449541292, Math.toRadians(90));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();

        Pose ParkPose = new Pose(114.3963133640553, 5.695852534562203, Math.toRadians(0));
        Pose pick2d = new Pose(133,36,Math.toRadians(0));
        Pose ShootPos = new Pose(83.20737327188938,19.94470046082949,Math.toRadians(0));
        Pose pick3rd = new Pose(130,9,Math.toRadians(0));
        Pose prePick3rd = new Pose(120.8617511520738,12,Math.toRadians(0));
        Pose pick3rdUp = new Pose(130,17,Math.toRadians(0));

        PathChain pick2nd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(77.68631336405532,40.635023041474646),
                                pick2d
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        PathChain shoot2nd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pick2d,
                                ShootPos
                        )
                ).setConstantHeadingInterpolation(pick2d.getHeading())
                .build();
        PathChain Pick3rd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPos,
                                pick3rdUp
                        )
                ).setConstantHeadingInterpolation(ShootPos.getHeading())
                .build();
        PathChain rePick3rd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pick3rd,
                                prePick3rd
                        )
                ).setConstantHeadingInterpolation(pick3rd.getHeading())
                .build();
        PathChain Pickag3rd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                prePick3rd,
                                pick3rd
                        )
                ).setConstantHeadingInterpolation(prePick3rd.getHeading())
                .build();

        PathChain Shoot3rd = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                pick3rd,
                                ShootPos
                        )
                ).setConstantHeadingInterpolation(pick3rd.getHeading())
                .build();



        PathChain park = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPos,
                                ParkPose
                        )
                )
                .setConstantHeadingInterpolation(
                        ShootPos.getHeading()
                )
                .build();


        autoCommand =
                new ParallelCommandGroup(
                        new intakeDefaultCMD(intakeSubsystem,spindexSubsystem),
                new SequentialCommandGroup(
                        new InstantCommand(()->shooterSubsystem.setTargetVelocity(1470)),
                        new TurretToPosCMD(turretSubsystem,0d),
                        new HookDownCMD(hookSubsystem),
                        new RunCommand(() ->
                                // eval obelisk here to store for the rest of the auto
                                spindexSubsystem.obeliskPattern = llSubsystem.getObelisk()
                        ).withTimeout(1100),
                        new ShootModeCMD(spindexSubsystem),
                        new TurretToPosCMD(turretSubsystem,19d),
                        new Shoot3BallsCMD(hookSubsystem,spindexSubsystem,()->spindexSubsystem.getPatternOffset()),
                        new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.42)),
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(pick2nd),
                                new SpindexModeDefaultCMD(spindexSubsystem),
                                new TurretToPosCMD(turretSubsystem,-70d),
                                new InstantCommand(()->shooterSubsystem.setTargetVelocity(1420))

                        ),
                        new WaitCommand(300),
                        new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                        new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(shoot2nd),
                                new SpindexModeDefaultCMD(spindexSubsystem)
                        ),

                        new WaitCommand(200),
                        new Shoot3BallsCMD(hookSubsystem,spindexSubsystem, ()->spindexSubsystem.getPatternOffset()),

                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(Pick3rd),
                                new SpindexModeDefaultCMD(spindexSubsystem)

                        ),
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(rePick3rd),
                                new SpindexModeDefaultCMD(spindexSubsystem)

                        ),
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(Pickag3rd),
                                new SpindexModeDefaultCMD(spindexSubsystem)
                        ),
                        new ParallelDeadlineGroup(
                                pedroSubsystem.followPathCmd(rePick3rd),
                                new SpindexModeDefaultCMD(spindexSubsystem)
                        ),
                        new WaitCommand(600),

                        new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Shoot3rd),
                        new SpindexModeDefaultCMD(spindexSubsystem)),

                        new WaitCommand(200),
                        new Shoot3BallsCMD(hookSubsystem,spindexSubsystem, ()->spindexSubsystem.getPatternOffset()),

                        new TurretToPosCMD(turretSubsystem,45d),
                        pedroSubsystem.followPathCmd(park)





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
        // 3) Telemetría para debug
        Pose pose = pedroSubsystem.follower.getPose();
        PedroSubsystem.robotPose = pedroSubsystem.follower.getPose();


        telemetry.addData("Follower busy", pedroSubsystem.follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
