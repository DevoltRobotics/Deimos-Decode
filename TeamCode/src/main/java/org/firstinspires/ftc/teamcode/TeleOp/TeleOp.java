package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookUpCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpinSorterCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.HookSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;

    @Override
    public void initialize() {
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);

        Garra.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new TurretAutoLLCMD(turretSubsystem, llSubsystem), new RunCommand(() -> {
            turretSubsystem.setTurretPower(gamepad1.left_stick_x);
        }));

        Garra.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new ShooterAutoLLCMD(shooterSubsystem, llSubsystem));


        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new IntakeHoldCMD(intakeSubsystem));

        Button IntakeIn = new GamepadButton(
                Garra, GamepadKeys.Button.A
        );

        IntakeIn.whenHeld(new IntakeInCMD(intakeSubsystem));
        IntakeIn.whenPressed(new InstantCommand(() -> spindexSubsystem.setShootMode(false)));

        Button IntakeOut = new GamepadButton(
                Garra, GamepadKeys.Button.B
        );

        IntakeOut.whenHeld(new IntakeOutCMD(intakeSubsystem));

        Button lastB = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_LEFT
        );

        lastB.whenPressed(new NextPosSorterCMD(spindexSubsystem, true));

        Button shootmode = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_UP
        );


        shootmode.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> spindexSubsystem.setShootMode(true)),
        new InstantCommand(() -> spindexSubsystem.lastFlickSeen = 0),
        new InstantCommand(() -> hookSubsystem.nFlick = 0)
        ));


        Button nextB = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_RIGHT
        );

        nextB.whenPressed(new NextPosSorterCMD(spindexSubsystem, true));

        Button intakemode = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_DOWN
        );


        intakemode.whenPressed(new InstantCommand(() -> spindexSubsystem.setShootMode(false)));

        intakemode.whenPressed(new ParallelCommandGroup(
                new InstantCommand(() -> spindexSubsystem.setShootMode(false)),
                        new InstantCommand(()->spindexSubsystem.setNBalls(-1))
        ));


        Button hookUpAndDown = new GamepadButton(
                Garra, GamepadKeys.Button.RIGHT_BUMPER
        );

        hookUpAndDown.whenPressed(new UpAndDownCMD(hookSubsystem));

        Button hookDown = new GamepadButton(
                Garra, GamepadKeys.Button.LEFT_BUMPER
        );

        hookDown.whenPressed(new HookDownCMD(hookSubsystem));


    }
}
