package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretPowerCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Autonomous(name = "RedA", group = "##", preselectTeleOp = "TeleOpRed")
public class RedA extends OpModeCommand {

    private Command autoCommand;

    public RedA() {
        super(Alliance.RED);
    }

    @Override
    public void initialize() {
        spindexSubsystem.setnBalls(3);
        Pose startPose = new Pose(123.90184049079755, 119.04294478527608, Math.toRadians(0));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();


        Pose ShootPos = new Pose(86.521, 83, Math.toRadians(0));

        Pose Pick1Cycle = new Pose(120.14723926380367, 77.61349693251533, 0);

        Pose P3rdCycle = new Pose(120, 56, 0);

        Pose P4thCycle = new Pose(120, 31.8, 0);

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
                                P3rdCycle,
                                new Pose(119.926, 59.190)
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
                                P4thCycle,
                                new Pose(119.043, 31.804)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        //TODO
        autoCommand = new SequentialCommandGroup(
                pedroSubsystem.followPathCmd(FShoot),

                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new HookDownCMD(hookSubsystem),
                                new RunCommand(() -> llSubsystem.getObelisk()).withTimeout(1500),

                                new TurretPowerCMD(turretSubsystem, -0.5).withTimeout(500), // turn slightly

                                ObeliskDecision(
                                        shootThree(SpindexSubsystem.ShootPos2), // GPP
                                        shootThree(SpindexSubsystem.ShootPos), // PGP
                                        shootThree(SpindexSubsystem.ShootPos3) // PPG
                                )
                        ),

                        new IntakeHoldCMD(intakeSubsystem)
                ),


                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                pedroSubsystem.followPathCmd(GoTo1Cycle),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(1)),
                pedroSubsystem.followPathCmd(Shoot2Cycle),

                ObeliskDecision(
                        shootThree(SpindexSubsystem.ShootPos3), // GPP
                        shootThree(SpindexSubsystem.ShootPos2), // PGP
                        shootThree(SpindexSubsystem.ShootPos) // PPG
                ),
                
                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                pedroSubsystem.followPathCmd(Pick3rdCycle),

                ObeliskDecision(
                        shootThree(SpindexSubsystem.ShootPos2), // GPP
                        shootThree(SpindexSubsystem.ShootPos), // PGP
                        shootThree(SpindexSubsystem.ShootPos3) // PPG
                ),

                new InstantCommand(() -> pedroSubsystem.follower.setMaxPower(0.5)),
                pedroSubsystem.followPathCmd(Pick4thCycle)
        );
    }

    public Command ObeliskDecision(Command GPP, Command PGP, Command PPG) {
        return new ConditionalCommand(
                GPP, // GPP
                new ConditionalCommand(
                        PGP, // PGP
                        PPG,// PPG
                        () -> llSubsystem.getObelisk() == Pattern.PGP
                ),
                () -> llSubsystem.getObelisk() == Pattern.GPP
        );
    }


    public Command shootThree(double spindexOffset) {
        return new ParallelDeadlineGroup(
                // DEADLINE: esta secuencia define cuánto dura todo
                new SequentialCommandGroup(
                        new WaitCommand(1500),
                        new Shoot3BallsCMD(hookSubsystem, spindexSubsystem, spindexOffset)
                ),

                new RunCommand(() -> telemetry.addData("Running", "shootThreeCmd at" + System.currentTimeMillis())),

                new TurretAutoLLCMD(turretSubsystem, llSubsystem),
                new ShooterAutoLLCMD(shooterSubsystem, llSubsystem)
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
    public void loop() {
        // 2) Correr el scheduler UNA vez por loop
        CommandScheduler.getInstance().run();

        // 3) Telemetría para debug
        Pose pose = pedroSubsystem.follower.getPose();
        telemetry.addData("Follower busy", pedroSubsystem.follower.isBusy());
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
