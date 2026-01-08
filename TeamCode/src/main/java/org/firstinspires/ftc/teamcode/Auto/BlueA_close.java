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
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.AutoIntakeModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexPosCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretPowerCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Autonomous(name = "Blue-cerca", group = "##", preselectTeleOp = "TeleOpBlue")
public class BlueA_close extends OpModeCommand {

    private Command autoCommand;


    public BlueA_close() {
        super(Alliance.BLUE);
    }

    @Override
    public void initialize() {
        spindexSubsystem.setTargetPos(SpindexSubsystem.ShootPos);
        turretSubsystem.resetEncoder();
        spindexSubsystem.SARSP();
        spindexSubsystem.setnBalls(3);
        Pose startPose = new Pose(19.8, 119.04294478527608, Math.toRadians(180));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();


        Pose ShootPos = new Pose(57.18084049079755, 83, Math.toRadians(180));

        Pose Pick1Cycle = new Pose(14.70184049079755, 88.56441717791411, Math.toRadians(180));

        Pose P3rdCycle = new Pose(7.5, 60.5, Math.toRadians(180));

        //Pose P4thCycle = new Pose(126, 30.7, 0);

        Pose P4thCycle = new Pose(57.20184049079755, 110, Math.toRadians(180));


        PathChain FShoot = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                ShootPos
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain GoTo1Cycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                ShootPos,
                                Pick1Cycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Shoot2Cycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(new BezierLine(
                        Pick1Cycle,
                        ShootPos
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain Pick3rdCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootPos,
                                new Pose(70.45398773006136, 57.423312883435585),
                                P3rdCycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        PathChain Shoot3rdCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                P3rdCycle,
                                ShootPos
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        PathChain Pick4thCycle = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootPos,
                                new Pose(67.06380368098159, 26.503067484662584),
                                P4thCycle
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shooterSubsystem.setDefaultCommand(new ShooterShootCmd(shooterSubsystem, 1200));

        //TODO
        autoCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                pedroSubsystem.followPathCmd(FShoot),
                        new TurretPowerCMD(turretSubsystem,-1).withTimeout(1550)
                        ),

                new HookDownCMD(hookSubsystem),
                new RunCommand(() ->
                        // eval obelisk here to store for the rest of the auto
                        spindexSubsystem.obeliskPattern = llSubsystem.getObelisk()
                ).withTimeout(800),
                new TurretPowerCMD(turretSubsystem,1).withTimeout(325),
                new TurretAutoLLCMD(turretSubsystem,llSubsystem).withTimeout(1500),
                shootThree(
                        SpindexSubsystem.ShootPos3, // GPP
                        SpindexSubsystem.ShootPos2, // PGP
                        SpindexSubsystem.ShootPos, // PPG
                        null, 1200
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.35)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(GoTo1Cycle),
                        new AutoIntakeModeCMD(spindexSubsystem),
                        new IntakeInCMD(intakeSubsystem)
                ),
                new WaitCommand(300),


                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Shoot2Cycle),
                        new IntakeOutCMD(intakeSubsystem)
                ),

                shootThree(
                        SpindexSubsystem.ShootPos2, // GPP
                        SpindexSubsystem.ShootPos, // PGP
                        SpindexSubsystem.ShootPos3,  // PPG
                        null, 1200
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.42)),
                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Pick3rdCycle),
                        new AutoIntakeModeCMD(spindexSubsystem),
                        new IntakeInCMD(intakeSubsystem)
                ),                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                new WaitCommand(200),

                new ParallelDeadlineGroup(
                        pedroSubsystem.followPathCmd(Shoot3rdCycle),
                        new IntakeOutCMD(intakeSubsystem)
                ),

                shootThree(
                        SpindexSubsystem.ShootPos, // GPP
                        SpindexSubsystem.ShootPos3, // PGP
                        SpindexSubsystem.ShootPos2, // PPG
                        null, 1200
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(700),
                        pedroSubsystem.followPathCmd(Pick4thCycle),
                        new SpindexPosCMD(spindexSubsystem,0)
                )
        );
    }

    public Command shootThree(double gppSpindexOffset, double pgpSpindexOffset, double ppgSpindexOffset, Double turretAngle, int shooterVelocity) {
        return new SequentialCommandGroup(
                // new TurretToPosCMD(turretSubsystem, turretAngle),
                new ShootModeCMD(spindexSubsystem),
                new WaitUntilCommand(() -> shooterSubsystem.getCurrentVelocity() >= shooterVelocity - 20),
                new InstantCommand(()-> spindexSubsystem.setnBalls(3)),

                new Shoot3BallsCMD(hookSubsystem, spindexSubsystem, () -> {
                    switch (spindexSubsystem.obeliskPattern) { // we should have already detected and stored Pattern by now
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

        telemetry.addData("patron",spindexSubsystem.obeliskPattern);
        telemetry.addData("Follower busy", pedroSubsystem.follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
    }
}

