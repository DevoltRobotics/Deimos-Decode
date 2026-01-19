package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretPowerCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
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
        turretSubsystem.resetEncoder();
        spindexSubsystem.setnBalls(3);
        Pose startPose = new Pose(80.58715596330276, 8.807339449541292, Math.toRadians(90));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();

        Pose targetPose = new Pose(111.92660550458716, 9.137614678899077, Math.toRadians(90));

        // Path que SÍ tiene distancia: de startPose -> targetPose
        PathChain path1 = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,      // antes empezabas aquí
                                targetPose      // ahora terminas aquí
                        )
                )
                .setConstantHeadingInterpolation(
                        startPose.getHeading()
                )
                .build();
        shooterSubsystem.setDefaultCommand(new ShooterShootCmd(shooterSubsystem, 1450));
        intakeSubsystem.setDefaultCommand(new IntakeHoldCMD(intakeSubsystem));


        // Script del auton: seguir path1 y luego hacer tus comandos
        autoCommand =

                new SequentialCommandGroup(
                new HookDownCMD(hookSubsystem),
                new RunCommand(() ->
                        // eval obelisk here to store for the rest of the auto
                        spindexSubsystem.obeliskPattern = llSubsystem.getObelisk()
                ).withTimeout(1200),
                new ShootModeCMD(spindexSubsystem),
                        new TurretToPosCMD(turretSubsystem,-90d,true).withTimeout(1200),
                        new TurretToPosCMD(turretSubsystem,22.5,false).withTimeout(4600),
                new Shoot3BallsCMD(hookSubsystem,spindexSubsystem,()->spindexSubsystem.getPatternOffset())
                ,pedroSubsystem.followPathCmd(path1)
        );
    }

    public Command obeliskPrepareCmd(PathChain shootPath) {
        return new SequentialCommandGroup(
                new HookDownCMD(hookSubsystem),
                new RunCommand(() -> llSubsystem.getObelisk()).withTimeout(1500),

                new ConditionalCommand(
                        moveAndShootThreeCmd(60, shootPath).asProxy(), // GPP
                        new ConditionalCommand(
                                moveAndShootThreeCmd(300, shootPath).asProxy(), // PGP
                                moveAndShootThreeCmd(180, shootPath).asProxy(),// PPG
                                () -> llSubsystem.getObelisk() == Pattern.PGP
                        ),
                        () -> llSubsystem.getObelisk() == Pattern.GPP
                )
        );
    }

    public Command moveAndShootThreeCmd(double offset, PathChain shootPath) {
        return new SequentialCommandGroup(
                // prepare to shoot


                new ParallelCommandGroup(
                        new TurretAutoLLCMD(turretSubsystem, llSubsystem),
                        new ShooterShootCmd(shooterSubsystem,()-> 1530),

                        new SequentialCommandGroup(
                                new WaitCommand(1500),

                                // shoot
                                new WaitCommand(1000),
                                new UpAndDownCMD(hookSubsystem,spindexSubsystem),
                                new WaitCommand(1000),
                                new UpAndDownCMD(hookSubsystem,spindexSubsystem),
                                new WaitCommand(1000),
                                new UpAndDownCMD(hookSubsystem,spindexSubsystem),
                                new WaitCommand(400),
                                pedroSubsystem.followPathCmd(shootPath)
                        )
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
