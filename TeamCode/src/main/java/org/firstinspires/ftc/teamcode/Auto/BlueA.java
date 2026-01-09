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
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Autonomous(name = "BlueA", group = "##", preselectTeleOp = "TeleOpBlue")
public class BlueA extends OpModeCommand {

    private Command autoCommand;


    public BlueA() {
        super(Alliance.BLUE);
    }

    @Override
    public void initialize() {
        spindexSubsystem.setTargetPos(SpindexSubsystem.ShootPos);
        spindexSubsystem.SARSP();
        turretSubsystem.resetEncoder();
        spindexSubsystem.setnBalls(3);
        // Pose inicial (ejemplo de Pedro)
        Pose startPose = new Pose(63.19266055045872, 8.366972477064223, Math.toRadians(180));
        pedroSubsystem.follower.setStartingPose(startPose);

        llSubsystem.setObeliskPipeline();

        // Pose objetivo usando los mismos números del ejemplo,
        // pero ahora sí como destino "real"
            Pose targetPose = new Pose(35.669724770642205, 20.69724770642202, Math.toRadians(0));

        // Path que SÍ tiene distancia: de startPose -> targetPose
        PathChain path1 = pedroSubsystem.follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,      // antes empezabas aquí
                                targetPose      // ahora terminas aquí
                        )
                )
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        targetPose.getHeading()
                )
                .build();

        // Script del auton: seguir path1 y luego hacer tus comandos
        autoCommand = new SequentialCommandGroup(
                new HookDownCMD(hookSubsystem),
                new WaitCommand(1500),
                new RunCommand(() ->
                        // eval obelisk here to store for the rest of the auto
                        spindexSubsystem.obeliskPattern = llSubsystem.getObelisk()
                ).withTimeout(800),
                new TurretPowerCMD(turretSubsystem,1).withTimeout(325),
                new TurretAutoLLCMD(turretSubsystem,llSubsystem).withTimeout(2000),
                shootThree(
                        SpindexSubsystem.ShootPos3, // GPP
                        SpindexSubsystem.ShootPos2, // PGP
                        SpindexSubsystem.ShootPos, // PPG
                        null, 1200
                ),pedroSubsystem.followPathCmd(path1)
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
