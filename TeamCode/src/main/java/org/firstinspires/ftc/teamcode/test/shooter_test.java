package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterShootCmd;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;


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

        pedroSubsystem.setDefaultCommand(pedroSubsystem.fieldCentricCmd(gamepad1,alliance));
        pedroSubsystem.follower.setPose(PedroSubsystem.robotPose);
        intakeSubsystem.setDefaultCommand(new IntakeHoldCMD(intakeSubsystem));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new TurretAutoOdoCMD(turretSubsystem),
                new TurretToPosCMD(turretSubsystem, 0d)
                );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new TurretToPosCMD(turretSubsystem, 100d)
        );

        

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new ShooterShootCmd(shooterSubsystem, () -> TV));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new ShooterAutoOdoCMD(shooterSubsystem,turretSubsystem));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new UpAndDownCMD(hookSubsystem,spindexSubsystem));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenActive(new NextPosSorterCMD(spindexSubsystem));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenActive(new ShootModeCMD(spindexSubsystem));
    }
}
