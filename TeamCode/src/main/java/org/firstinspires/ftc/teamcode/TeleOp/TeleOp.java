package org.firstinspires.ftc.teamcode.TeleOp;

import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.commands.intake.intakeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.lift.LiftUpCMD;
import org.firstinspires.ftc.teamcode.commands.light.SetAutoStatus;
import org.firstinspires.ftc.teamcode.commands.shooter.ShooterAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.NextPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.compound.Shoot3BallsCMD;
import org.firstinspires.ftc.teamcode.commands.hook.HookDownCMD;
import org.firstinspires.ftc.teamcode.commands.hook.UpAndDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeHoldCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeInCMD;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOutCMD;
import org.firstinspires.ftc.teamcode.commands.intake.LiftDownCMD;
import org.firstinspires.ftc.teamcode.commands.intake.LiftHoldCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.IntakeModeCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.LastPosSorterCMD;
import org.firstinspires.ftc.teamcode.commands.sorter.SpindexModeDefaultCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretAutoOdoCMD;
import org.firstinspires.ftc.teamcode.commands.turret.TurretToPosCMD;
import org.firstinspires.ftc.teamcode.config.OpModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.PedroSubsystem;

public abstract class TeleOp extends OpModeCommand {

    GamepadEx Chasis;
    GamepadEx Garra;

    public TeleOp(Alliance alliance) {
        super(alliance);
    }


    @Override
    public void initialize() {
        pedroSubsystem.follower.setPose(PedroSubsystem.robotPose);
        Chasis = new GamepadEx(gamepad1);
        Garra = new GamepadEx(gamepad2);
        

        new HookDownCMD(hookSubsystem).schedule();
        new TurretToPosCMD(turretSubsystem,0d).schedule();

        shooterSubsystem.setDefaultCommand(new ShooterAutoOdoCMD(shooterSubsystem,turretSubsystem));
        spindexSubsystem.setDefaultCommand(new SpindexModeDefaultCMD(spindexSubsystem));
        statusLightSubsystem.setDefaultCommand(new SetAutoStatus(statusLightSubsystem,spindexSubsystem));

        pedroSubsystem.setDefaultCommand(pedroSubsystem.fieldCentricCmd(gamepad1, alliance));

        liftSubsystem.setDefaultCommand(new LiftHoldCMD(liftSubsystem));


        intakeSubsystem.setDefaultCommand(new intakeDefaultCMD(intakeSubsystem,spindexSubsystem));

        Button IntakeIn = new GamepadButton(
                Garra, GamepadKeys.Button.A
        );


        IntakeIn.whenHeld(new IntakeInCMD(intakeSubsystem,spindexSubsystem));

        Button Turretauto = new GamepadButton(
                Garra, GamepadKeys.Button.X
        );


        Turretauto.toggleWhenPressed(
              new TurretAutoOdoCMD(turretSubsystem)
        );




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

    @Override
    public void run(){
        PedroSubsystem.robotPose = pedroSubsystem.follower.getPose();

    }
}
