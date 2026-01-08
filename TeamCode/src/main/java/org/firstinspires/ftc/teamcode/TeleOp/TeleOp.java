package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.intake.LiftDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.LiftHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.LiftUpCMD;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoLLCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.IntakeModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.LastPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.ShootModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexModeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.transfer.StopTransferCMD;
import org.firstinspires.ftc.teamcode.commands.transfer.TransferCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoLLCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;

public abstract class TeleOp extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;

    public TeleOp(Alliance alliance) {
        super(alliance);
    }


    @Override
    public void initialize() {
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);

        Garra.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(
               new TransferCMD(transferSubsystem),new StopTransferCMD(transferSubsystem)
       );

        new HookDownCMD(hookSubsystem).schedule();

        shooterSubsystem.setDefaultCommand(new ShooterAutoLLCMD(shooterSubsystem,llSubsystem));
        turretSubsystem.setDefaultCommand(new TurretAutoLLCMD(turretSubsystem,llSubsystem));

        spindexSubsystem.setDefaultCommand(new SpindexModeDefaultCMD(spindexSubsystem));

        pedroSubsystem.setDefaultCommand(pedroSubsystem.fieldCentricCmd(gamepad1));

        liftSubsystem.setDefaultCommand(new LiftHoldCMD(liftSubsystem));


        Garra.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new ShooterAutoLLCMD(shooterSubsystem, llSubsystem));


        intakeSubsystem.setDefaultCommand(new IntakeHoldCMD(intakeSubsystem));

        Button IntakeIn = new GamepadButton(
                Garra, GamepadKeys.Button.A
        );

        IntakeIn.whenHeld(new IntakeInCMD(intakeSubsystem));

        Button IntakeOut = new GamepadButton(
                Garra, GamepadKeys.Button.B
        );

        IntakeOut.whenHeld(new IntakeOutCMD(intakeSubsystem));

        Button Uprobot = new GamepadButton(
                Chasis, GamepadKeys.Button.Y
        );

        Uprobot.whenHeld(new LiftUpCMD(liftSubsystem));

        Button DownRobot = new GamepadButton(
                Chasis, GamepadKeys.Button.B
        );

        DownRobot.whenHeld(new LiftDownCMD(liftSubsystem));

        Button lastB = new GamepadButton(
                Garra, GamepadKeys.Button.LEFT_BUMPER
        );

        lastB.whenPressed(new LastPosSorterCMD(spindexSubsystem));




        Button nextB = new GamepadButton(
                Garra, GamepadKeys.Button.RIGHT_BUMPER
        );

        nextB.whenActive(new NextPosSorterCMD(spindexSubsystem), false);

        Button intakemode = new GamepadButton(
                Garra, GamepadKeys.Button.DPAD_DOWN
        );

        intakemode.whenPressed(
                new IntakeModeCMD(spindexSubsystem)
        );

        Button AutoShoot = new GamepadButton(
                Garra, GamepadKeys.Button.Y
        );

        AutoShoot.whenPressed(
                new Shoot3BallsCMD(hookSubsystem, spindexSubsystem,()->spindexSubsystem.getPatternOffset())
        );


        Trigger hookUpAndDown = new Trigger(() -> gamepad2.right_trigger >= 0.1);

        hookUpAndDown.whenActive(new UpAndDownCMD(hookSubsystem, spindexSubsystem));




        Trigger hookDown = new Trigger(() -> gamepad2.left_trigger >= 0.1);

        hookDown.whenActive(new HookDownCMD(hookSubsystem));


    }
}
