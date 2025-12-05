package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Pattern;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;

@Autonomous(name = "BlueA", group = "##", preselectTeleOp = "TeleOpBlue")
public class BlueA extends OpModeCommand {

    private Command autoCommand;

    public BlueA() {
        super(Alliance.BLUE);
    }

    @Override
    public void initialize() {
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
        autoCommand = new ParallelCommandGroup(
                 obeliskPrepareCmd(path1),
                new IntakeHoldCMD(intakeSubsystem));
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
