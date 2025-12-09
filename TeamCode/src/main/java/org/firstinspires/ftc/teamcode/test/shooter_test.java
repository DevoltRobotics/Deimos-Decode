package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;


@Config
@TeleOp(name = "shooter test")
public class shooter_test extends OpModeCommand {

    GamepadEx gamepadEx1;
    public static double TV = 0;

    public shooter_test() {
        super(Alliance.ANY);
    }

    @Override
    public void initialize() {
        new RunCommand(()->{
            telemetry.update();
        }).schedule();

        gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new TurretAutoLLCMD(turretSubsystem, llSubsystem),
                new TurretToPosCMD(turretSubsystem, 300)
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenActive(new InstantCommand(() -> {
            turretSubsystem.resetRelative();
        }));

        new RunCommand(() -> {
            if(Math.abs(gamepad1.left_stick_x) > 0.2 && turretSubsystem.getCurrentCommand() != turretSubsystem.getDefaultCommand()) {
                turretSubsystem.getCurrentCommand().cancel();
            }
        }).schedule();

        RunCommand manualTurret = new RunCommand(() -> turretSubsystem.setTurretPower(gamepad1.left_stick_x));
        manualTurret.addRequirements(turretSubsystem);

        turretSubsystem.setDefaultCommand(manualTurret);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new ShooterShootCmd(shooterSubsystem, () -> TV));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new ShooterAutoLLCMD(shooterSubsystem,llSubsystem));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new UpAndDownCMD(hookSubsystem,spindexSubsystem));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenActive(new NextPosSorterCMD(spindexSubsystem));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenActive(new ShootModeCMD(spindexSubsystem));
    }
}
