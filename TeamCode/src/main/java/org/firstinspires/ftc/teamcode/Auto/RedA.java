package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.AutoIntakeModeCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretPowerCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Autonomous(name = "RedA", group = "##", preselectTeleOp = "TeleOpRed")
public class RedA extends OpModeCommand {

    private Command autoCommand;

    Pattern obeliskPattern;

    public RedA() {
        super(Alliance.RED);
    }

    @Override
    public void initialize() {
        turretSubsystem.resetRelative();
        spindexSubsystem.setnBalls(3);
        Pose startPose = new Pose(123.90184049079755, 119.04294478527608, Math.toRadians(0));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();


        Pose ShootPos = new Pose(86.521, 83, Math.toRadians(0));

        Pose Pick1Cycle = new Pose(123, 77.61349693251533, 0);

        Pose P3rdCycle = new Pose(123, 54.8, 0);

        Pose P4thCycle = new Pose(126, 30.7, 0);

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
                        new BezierCurve(
                                ShootPos,
                                new Pose(76.63803680981596, 26.503067484662584),
                                P4thCycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        shooterSubsystem.setDefaultCommand(new ShooterShootCmd(shooterSubsystem, 1200));

        //TODO
        autoCommand = new SequentialCommandGroup(
                pedroSubsystem.followPathCmd(FShoot),

                new HookDownCMD(hookSubsystem),
                new RunCommand(() ->
                        // eval obelisk here to store for the rest of the auto
                        obeliskPattern = llSubsystem.getObelisk()
                ).withTimeout(800),

                shootThree(
                        SpindexSubsystem.ShootPos2, // GPP
                        SpindexSubsystem.ShootPos, // PGP
                        SpindexSubsystem.ShootPos3, // PPG
                        235.0, 1200
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(GoTo1Cycle),
                        new AutoIntakeModeCMD(spindexSubsystem),
                        new IntakeInCMD(intakeSubsystem)
                ),


                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                pedroSubsystem.followPathCmd(Shoot2Cycle),

                shootThree(
                        SpindexSubsystem.ShootPos3, // GPP
                        SpindexSubsystem.ShootPos2, // PGP
                        SpindexSubsystem.ShootPos,  // PPG
                        null, 1200
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Pick3rdCycle),
                        new AutoIntakeModeCMD(spindexSubsystem),
                        new IntakeInCMD(intakeSubsystem)
                ),                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                pedroSubsystem.followPathCmd(Shoot3rdCycle),

                shootThree(
                        SpindexSubsystem.ShootPos2, // GPP
                        SpindexSubsystem.ShootPos, // PGP
                        SpindexSubsystem.ShootPos3, // PPG
                        null, 1200
                ),
                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                pedroSubsystem.followPathCmd(Pick4thCycle),
                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1))
        );
    }

    public Command shootThree(double gppSpindexOffset, double pgpSpindexOffset, double ppgSpindexOffset, Double turretAngle, int shooterVelocity) {
        return new SequentialCommandGroup(
                new TurretToPosCMD(turretSubsystem, turretAngle),
                new WaitUntilCommand(() -> shooterSubsystem.getCurrentVelocity() >= shooterVelocity - 20),
                new InstantCommand(()-> spindexSubsystem.setnBalls(3)),

                new Shoot3BallsCMD(hookSubsystem, spindexSubsystem, () -> {
                    switch (obeliskPattern) { // we should have already detected and stored Pattern by now
                        case UNKNOWN: // when unknown, return GPP
                        case GPP:
                            return gppSpindexOffset;
                        case PGP:
                            return pgpSpindexOffset;
                        case PPG:
                            return ppgSpindexOffset;
                    }

                    return gppSpindexOffset; // when unknown, return GPP
                }),

                new InstantCommand(() -> log(2000, "shootThree", "Done with sequence"))
        ).raceWith(
                new IntakeHoldCMD(intakeSubsystem),
                new ShooterShootCmd(shooterSubsystem, shooterVelocity).asProxy()
        );
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

        telemetry.addData("Follower busy", pedroSubsystem.follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
    }
}
